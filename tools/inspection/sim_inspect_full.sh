#!/bin/bash
# 明日の一連の流れ（マーカー接近 → 立ったままアーム → 撮影 → 収納）を、実機なしで通す（2026-09-28）。
#
# 実機と**同じコード**を動かす: approach_node（marker_approach）と inspect_run.py。
# 代役にするのは、機体・検出器・温度（sim_world_node.py）、アーム（sim_fakes/d1_fake.py）、
# 撮影（sim_fakes/rs_snap_fake.sh）だけ。
#
# 実機の DDS に混ざらないよう ROS_DOMAIN_ID=77・ROS_LOCALHOST_ONLY=1 で閉じる。
#
# 使い方:
#   tools/sim_inspect_full.sh                       # 既定: マーカーから1.3m・横0・向き0・視線接近
#   tools/sim_inspect_full.sh 1.3 0.02 3 5          # 距離[m] 横[m] 向き[度] seed
#   USE_NORMAL=true tools/sim_inspect_full.sh       # 法線接近で比べる
#   POSTURE=stand tools/sim_inspect_full.sh         # 立ったままアーム（既定は lie: 到着後に伏せて撮り、起立して終わる）
#   INSPECT_ARGS="--fast-path" tools/sim_inspect_full.sh   # inspect_run.py に足す引数
#   D1_FAKE_DROP=1 tools/sim_inspect_full.sh        # アームの代役が最初の指令を捨てる（届かない再現）
#   SIM_RENDER=1 SIM_ROOT_DY=0.06 INSPECT_ARGS="--aim-correct" tools/sim_inspect_full.sh   # 撮影位置の補正（部材を6cm遠くに）
set -u
DIST=${1:-1.3}; LAT=${2:-0.0}; YAW=${3:-0.0}; SEED=${4:-0}
USE_NORMAL=${USE_NORMAL:-false}
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$HOME/marker_detection/logs/sim
mkdir -p "$OUT"
TAG=$(date +%m%d_%H%M%S)
RES=$OUT/full_${TAG}.json
export D1_FAKE_STATE=$OUT/d1_fake_state_${TAG}.json

set +u
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/install/setup.bash" 2>/dev/null || source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/marker_detection/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI

pids=()
cleanup() {
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 1
  for p in "${pids[@]}"; do kill -9 "$p" 2>/dev/null; done
  rm -f "$D1_FAKE_STATE" "$HOME"/marker_detection/logs/rs/simfull_*_{color.png,depth.pgm,info.txt,root.png}
}
trap cleanup EXIT

python3 "$HERE/sim_world_node.py" --dist "$DIST" --lat "$LAT" --yaw "$YAW" --seed "$SEED" \
  --hip-start "${HIP_START:-40}" --hip-rate "${HIP_RATE:-2}" \
  --out "$RES" > "$OUT/world_${TAG}.log" 2>&1 &
pids+=($!)
# approach_real.launch.py と同じ設定（camera_x/z、turn_lead 6.9、settle 0.7）
# ros2 run 経由だと、止めるときに ros2 run だけが死んでノードが残る（実際に8本残った）。直接起動する
"$HOME/marker_detection/ros2_ws/install/marker_approach/lib/marker_approach/approach_node" --ros-args \
  -p dry_run:=false -p standoff:=0.65 -p use_normal:=$USE_NORMAL -p settle_time:=0.7 \
  -p camera_x:=0.333 -p camera_y:=0.0 -p camera_z:=0.035 -p camera_pitch_deg:=0.0 \
  -p turn_lead_angle_deg:=6.9 -p trace_csv:="$OUT/trace_${TAG}.csv" \
  -r marker_pose:=/marker_detector_node/pose \
  -r marker_diagnostics:=/marker_detector_node/diagnostics > "$OUT/approach_${TAG}.log" 2>&1 &
pids+=($!)
sleep 4

python3 "$HERE/inspect_run.py" --approach marker --posture "${POSTURE:-lie}" --auto --hold 1 --name simfull \
  --d1-run "$HERE/sim_fakes/d1_fake.py" --rs-snap "$HERE/sim_fakes/rs_snap_fake.sh" ${INSPECT_ARGS:-} </dev/null
rc=$?
# 実機の記録（logs/inspect_*.log）と混ざらないよう、今回の記録を logs/sim へ移す
last=$(ls -t "$HOME"/marker_detection/logs/inspect_*.log 2>/dev/null | head -1)
[ -n "$last" ] && grep -q "name simfull\|simfull" "$last" && mv "$last" "$OUT/inspect_sim_${TAG}.log"
echo
echo "== inspect_run の終了コード: $rc"
echo "== 接近ノードの最後のログ:"
grep -E "停止しました|打ち切り|== " "$OUT/approach_${TAG}.log" | tail -6
echo "== 真値（仮想世界）:"
for _ in $(seq 20); do [ -f "$RES" ] && break; sleep 0.5; done
cat "$RES" 2>/dev/null || echo "(結果なし)"
echo
exit $rc
