#!/bin/bash
# 実機用の一連ツール(tools/inspection/inspect_run.py)を、go2-sim コンテナの中だけで通し試験する(#83)。
#
#   マーカー接近 → 伏せる → アーム展開 → 撮影 → 収納 → 起立
#
# 実機と同じ approach_node と inspect_run.py を動かし、機体・検出器・温度(sim_world_node.py)、
# アーム(sim_fakes/d1_fake.py)、撮影(sim_fakes/rs_snap_fake.sh)だけを代役にする。Gazebo は起動しない。
# ホスト側に ROS も ~/marker_detection も要らない（イメージのビルドだけ済んでいればよい）。
#
# 使い方:
#   docker/sim/tools/inspect_sim.sh                       # 既定: マーカーから1.3m・横0・向き0
#   docker/sim/tools/inspect_sim.sh 1.3 0.02 3 5          # 距離[m] 横[m] 向き[度] seed
#   POSTURE=stand docker/sim/tools/inspect_sim.sh         # 立ったままアーム（既定は伏せて撮り、起立して終わる）
#   INSPECT_ARGS="--fast-path" docker/sim/tools/inspect_sim.sh
#   SIM_RENDER=1 SIM_ROOT_DY=0.06 INSPECT_ARGS="--aim-correct" docker/sim/tools/inspect_sim.sh
# 他に D1_FAKE_DROP / USE_NORMAL / HIP_START / HIP_RATE も sim_inspect_full.sh へそのまま渡る。
# 記録は終了時にコンテナごと消える。残したいときは SIM_OUT_HOST=<ホストのディレクトリ> を付ける。
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
cd "$HERE/.."

envs=()
for v in POSTURE INSPECT_ARGS USE_NORMAL SIM_RENDER SIM_ROOT_DY D1_FAKE_DROP HIP_START HIP_RATE; do
  [ -n "${!v:-}" ] && envs+=(-e "$v=${!v}")
done
vols=()
if [ -n "${SIM_OUT_HOST:-}" ]; then
  mkdir -p "$SIM_OUT_HOST"
  vols+=(-v "$(cd "$SIM_OUT_HOST" && pwd):/tmp/inspect_sim")
fi

# Gazebo の起動コマンドは使わず、通し試験だけを走らせる(--no-deps)。コンテナは使い捨て
exec docker compose run --rm -T --no-deps "${envs[@]}" "${vols[@]}" sim \
  bash -c '/inspection/sim_inspect_full.sh "$@"' _ "$@"
