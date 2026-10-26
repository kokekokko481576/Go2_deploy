#!/bin/bash
# Gazebo で横一線スイープ+aim_correctの統合（4点、実際の深度画像）を流して録画する。
# run_demo_1007.sh の写し。違いは sweep_demo.py / recorder_sweep.py を使うことと、
# 部材はずらさず既定位置に置くこと（--prepareや秒数指定には対応していない）。
#
# 前提: go2-sim コンテナが起動していること（~/marker_detection は要らない。必要なものは compose.yaml が
#       /ros2_ws・/sim_tools・/inspection にマウントする、#83）。Nav2 は切っておく:
#   cd ~/bridge/Go2_deploy/docker/sim && xhost +local:docker && SIM_ENABLE_NAV2=false SIM_ENABLE_RVIZ=false docker compose up -d
#
# 使い方: docker/sim/tools/demo/run_demo_sweep.sh
# 出力: $DEMO_OUT（既定 ~/go2_sim_demo）/Go2_sim_sweep_<日時>.mp4、写真 $DEMO_OUT/sweep_<日時>/
set -u
C=go2-sim
TAG=$(date +%m%d_%H%M%S)
DEMO_OUT=${DEMO_OUT:-$HOME/go2_sim_demo}; mkdir -p "$DEMO_OUT"
HERE=$(cd "$(dirname "$0")" && pwd)
R() { docker exec "$C" bash -c ". /opt/ros/jazzy/setup.bash && source \$WORKSPACE_DIR/install/setup.bash && $1"; }
RD() { docker exec -d "$C" bash -c ". /opt/ros/jazzy/setup.bash && source \$WORKSPACE_DIR/install/setup.bash && $1"; }
kill_ours() {
  docker exec "$C" bash -c 'ps -ef | grep -E "[a]priltag_node|[s]im_tag_bridge|[m]arker_approach.approach_node|[r]ecorder.py|[s]weep_demo.py" | awk "{print \$2}" | xargs -r kill -INT' 2>/dev/null
  sleep 2
  docker exec "$C" bash -c 'ps -ef | grep -E "[a]priltag_node|[s]im_tag_bridge|[m]arker_approach.approach_node|[s]weep_demo.py" | awk "{print \$2}" | xargs -r kill -9' 2>/dev/null
}
kill_ours
# 日本語フォント（字幕用。イメージに fonts-noto-cjk が入っている）
docker exec "$C" bash -c 'mkdir -p /tmp/fonts && ln -sf /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc /tmp/fonts/'
# 部材と録画用カメラ(既定位置。ずらさない)
"$HERE/spawn_scene.sh"
# 出発位置: マーカー(-4.96,1.5)の手前1.3m、中心線上、マーカーを向く
R 'gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
   --req "name: \"robot1_my_bot\", position: {x: -3.66, y: 1.5, z: 0.45}, orientation: {x: 0, y: 0, z: 1, w: 0}"' >/dev/null
R 'ros2 topic pub -t 3 /robot1/cmd_vel geometry_msgs/msg/Twist "{}"' >/dev/null 2>&1
# 止まって立った状態で始める（アームを動かしている間は歩かない。歩き出すのは sweep_demo.py が収納を終えてから）
R 'for m in REST; do ros2 topic pub -t 2 /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand "{mode: $m, robot_id: 1}"; done' >/dev/null 2>&1
# 検出（sim_up.sh と同じ値）と接近制御（ホーム側の最新＝実機と同じ。simの視野±31度に合わせて予算を絞る）
RD "ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/robot1/color/image_raw -r camera_info:=/robot1/color/camera_info \
    -p family:=36h11 -p size:=0.078434 -p detector.decimate:=1.0 -p use_sim_time:=true > /tmp/apriltag.log 2>&1"
RD "python3 /sim_tools/sim_tag_bridge.py > /tmp/bridge.log 2>&1"
RD "export PYTHONPATH=/ros2_ws/src/marker_approach:\$PYTHONPATH && python3 -m marker_approach.approach_node --ros-args \
    -r cmd_vel_raw:=/robot1/cmd_vel -p camera_x:=0.33 -p camera_y:=0.0 -p camera_z:=0.0057 \
    -p standoff:=0.65 -p dry_run:=false -p use_normal:=false \
    -p fov_budget_deg:=12.0 -p drive_bearing_limit_deg:=18.0 -p max_runtime:=150.0 > /tmp/approach.log 2>&1"
sleep 6
# 録画を始めてから流す(sweep_demo.pyは--prepareに対応していないので、ここで直接本番を回す)
docker exec "$C" rm -rf /tmp/sweep_shots /tmp/sweep_aim
RD "python3 /sim_tools/demo/recorder_sweep.py /tmp/demo_$TAG.mp4 > /tmp/recorder.log 2>&1"
sleep 3
R "set -o pipefail; python3 /sim_tools/demo/sweep_demo.py 2>&1 | tee /tmp/sweep_demo.log"
rc=$?
# 録画は「完了」の3秒後に自分で書き終える。念のため待つ
for _ in $(seq 20); do docker exec "$C" bash -c 'ps -ef | grep -q "[r]ecorder.py"' || break; sleep 1; done
kill_ours
docker exec "$C" tail -1 /tmp/recorder.log
OUT="$DEMO_OUT/Go2_sim_sweep_$TAG.mp4"
docker cp "$C":/tmp/demo_$TAG.mp4 "$OUT" && echo "動画: $OUT"
D="$DEMO_OUT/sweep_$TAG"; mkdir -p "$D"
docker cp "$C":/tmp/sweep_shots/. "$D/" 2>/dev/null; docker cp "$C":/tmp/sweep_aim/. "$D/aim/" 2>/dev/null
docker cp "$C":/tmp/sweep_demo.log "$D/" 2>/dev/null; echo "写真・試し撮り・ログ: $D/"
echo "接近のログ: docker exec $C tail -20 /tmp/approach.log"
exit $rc
