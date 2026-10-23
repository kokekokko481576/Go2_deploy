#!/bin/bash
# Gazebo で 10/7 の3つの変更（①収納を真後ろへ ②3段で展開 ③深度で撮影位置を補正）を流して録画する。
# run_demo.sh（9/29）の写し。違いは demo_1007.py / recorder_1007.py を使うことと、部材をずらして置くこと。
#   MEMBER_DY=0.07 MEMBER_DX=0.05 が既定（機体から7cm遠く・5cm前へ。止まる位置のばらつきの再現）
#
# 前提: go2-sim コンテナが起動していること。~/marker_detection が /marker_detection に読み取り専用で
#       マウントされていること（docker/sim/compose.override.yaml）。Nav2 は切っておく:
#   cd ~/bridge/Go2_deploy/docker/sim && xhost +local:docker && SIM_ENABLE_NAV2=false SIM_ENABLE_RVIZ=false docker compose up -d
#
# 使い方: ~/marker_detection/tools/sim_gz/run_demo_1007.sh [撮影姿勢で待つ秒数]
# 出力: ~/デスクトップ/Go2_sim_1007_<日時>.mp4、写真 ~/marker_detection/logs/sim/demo1007_<日時>/
set -u
C=go2-sim
HOLD=${1:-2}
TAG=$(date +%m%d_%H%M%S)
HERE=$(cd "$(dirname "$0")" && pwd)
R() { docker exec "$C" bash -c ". /opt/ros/jazzy/setup.bash && source \$WORKSPACE_DIR/install/setup.bash && $1"; }
RD() { docker exec -d "$C" bash -c ". /opt/ros/jazzy/setup.bash && source \$WORKSPACE_DIR/install/setup.bash && $1"; }
kill_ours() {
  docker exec "$C" bash -c 'ps -ef | grep -E "[a]priltag_node|[s]im_tag_bridge|[m]arker_approach.approach_node|[r]ecorder.py|[d]emo_flow.py" | awk "{print \$2}" | xargs -r kill -INT' 2>/dev/null
  sleep 2
  docker exec "$C" bash -c 'ps -ef | grep -E "[a]priltag_node|[s]im_tag_bridge|[m]arker_approach.approach_node|[d]emo_flow.py" | awk "{print \$2}" | xargs -r kill -9' 2>/dev/null
}
kill_ours
# 日本語フォント（字幕用。コンテナには入っていない）
docker exec "$C" mkdir -p /tmp/fonts && docker cp /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc "$C":/tmp/fonts/ >/dev/null
# 部材と録画用カメラ
MEMBER_DY=${MEMBER_DY:-0.07} MEMBER_DX=${MEMBER_DX:-0.05} "$HERE/spawn_scene.sh"
# 出発位置: マーカー(-4.96,1.5)の手前1.3m、中心線上、マーカーを向く
R 'gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
   --req "name: \"robot1_my_bot\", position: {x: -3.66, y: 1.5, z: 0.45}, orientation: {x: 0, y: 0, z: 1, w: 0}"' >/dev/null
R 'ros2 topic pub -t 3 /robot1/cmd_vel geometry_msgs/msg/Twist "{}"' >/dev/null 2>&1
# 止まって立った状態で始める（アームを動かしている間は歩かない。歩き出すのは demo_1007.py が収納を終えてから）
R 'for m in REST; do ros2 topic pub -t 2 /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand "{mode: $m, robot_id: 1}"; done' >/dev/null 2>&1
# 検出（sim_up.sh と同じ値）と接近制御（ホーム側の最新＝実機と同じ。simの視野±31度に合わせて予算を絞る）
RD "ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/robot1/color/image_raw -r camera_info:=/robot1/color/camera_info \
    -p family:=36h11 -p size:=0.078434 -p detector.decimate:=1.0 -p use_sim_time:=true > /tmp/apriltag.log 2>&1"
RD "python3 /marker_detection/tools/sim_tag_bridge.py > /tmp/bridge.log 2>&1"
RD "export PYTHONPATH=/marker_detection/ros2_ws/src/marker_approach:\$PYTHONPATH && python3 -m marker_approach.approach_node --ros-args \
    -r cmd_vel_raw:=/robot1/cmd_vel -p camera_x:=0.33 -p camera_y:=0.0 -p camera_z:=0.0057 \
    -p standoff:=0.65 -p dry_run:=false -p use_normal:=false \
    -p fov_budget_deg:=12.0 -p drive_bearing_limit_deg:=18.0 -p max_runtime:=150.0 > /tmp/approach.log 2>&1"
sleep 6
# 録画を始めてから流す
# 録画の前に体を上げ切り、アームを従来の収納にしておく（動画は ① から始める）
R "python3 /marker_detection/tools/sim_gz/demo_1007.py --prepare" >/dev/null 2>&1
docker exec "$C" rm -rf /tmp/demo_shots /tmp/demo_aim
RD "python3 /marker_detection/tools/sim_gz/recorder_1007.py /tmp/demo_$TAG.mp4 > /tmp/recorder.log 2>&1"
sleep 3
R "set -o pipefail; python3 /marker_detection/tools/sim_gz/demo_1007.py $HOLD 2>&1 | tee /tmp/demo_1007.log"
rc=$?
# 録画は「完了」の3秒後に自分で書き終える。念のため待つ
for _ in $(seq 20); do docker exec "$C" bash -c 'ps -ef | grep -q "[r]ecorder.py"' || break; sleep 1; done
kill_ours
docker exec "$C" tail -1 /tmp/recorder.log
OUT="$HOME/デスクトップ/Go2_sim_1007_$TAG.mp4"
docker cp "$C":/tmp/demo_$TAG.mp4 "$OUT" && echo "動画: $OUT"
D="$HOME/marker_detection/logs/sim/demo1007_$TAG"; mkdir -p "$D"
docker cp "$C":/tmp/demo_shots/. "$D/" 2>/dev/null; docker cp "$C":/tmp/demo_aim/. "$D/aim/" 2>/dev/null
docker cp "$C":/tmp/demo_1007.log "$D/" 2>/dev/null; echo "写真・試し撮り・ログ: $D/"
echo "接近のログ: docker exec $C tail -20 /tmp/approach.log"
exit $rc
