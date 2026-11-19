#!/bin/bash
# 実機ライブを「9月構成」(9/4、コミット88879e2相当)に切り替える。
# 2026-11-16に手打ちでやって過去最高の地図(walk_1116e)が出た手順を固定したもの。
# 2026-11-18: 床除去の floor_margin を9月と同じ0.3にした。9月は設定ファイル経由で0.3が
# 効いていたが、11/16と11/18午前の「9月構成」は引数が抜けて既定値0.1で動いていた。
#
# 前提: ホストで ./docker/driver/real_up.sh up 済み。devコンテナ(arbeit-ros2)の中で実行する。
#   docker exec -it arbeit-ros2 bash /home/ros/ros2_ws/bags/tools/live_sept.sh start
#
#   start [ang] : EKF・height_slice_viz・slam_toolboxを止めて9月版で起動し直す(地図は新規)
#   reset [ang] : slam_toolboxだけ再起動して地図をクリアする
#                 ang を付けると角度探索を狭めた設定を足す(reset でも毎回付けること)
#   status : 3ノードの起動引数を表示する
#
# 起動するのは観測系ノードだけで、機体を動かす経路には触らない。
# 注意: killはコンテナ内の同名プロセス全部が対象(ドメインを見ない)。別ドメインで
#       オフライン再生を回している最中には使わないこと。
# (set -u は付けない: ROSのsetup.bashが未定義変数を参照して落ちる)
T=/home/ros/ros2_ws/bags/tools/sept
LOG=/tmp/live_sept
mkdir -p "$LOG"
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash

# 第2引数に ang を付けると、slam_toolboxの角度探索を±6度に狭めてオドメトリの向きを信用させる
# (2026-11-18、walk_1118dのオフライン再生で8度超の飛びが21回→0回になった設定。9月構成には無い)
SLAM_EXTRA=""
if [ "${2:-}" = ang ]; then
  SLAM_EXTRA="-p coarse_search_angle_offset:=0.105 -p coarse_angle_resolution:=0.0175 -p angle_variance_penalty:=0.35 -p minimum_angle_penalty:=0.7 -p use_response_expansion:=false"
fi

kill_pat() { pgrep -f "$1" | xargs -r kill -9 2>/dev/null; }

start_slam() {
  kill_pat 'lib/slam_toolbox/'
  sleep 1
  nohup /opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node --ros-args \
    --params-file "$T/slam_toolbox_real.yaml" -p use_sim_time:=false \
    $SLAM_EXTRA -r map:=/go2_localization/map > "$LOG/slam.log" 2>&1 &
}

status() {
  ps -eo pid,etime,args | grep -E '[e]kf_node|[h]eight_slice_viz|[a]sync_slam_toolbox_node|[p]ointcloud_to_laserscan_node|[s]lam_checkpoint' | cut -c1-260
}

case "${1:-}" in
  start)
    kill_pat 'robot_localization/ekf_node'
    kill_pat 'height_slice_viz'
    sleep 1
    nohup /opt/ros/humble/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node \
      --params-file "$T/ekf.yaml" -p use_sim_time:=false \
      -r raw_odom_input:=/go2_state_bridge/odom -r imu_plugin/out:=/go2_state_bridge/imu \
      -r odometry/filtered:=/go2_localization/odometry/filtered > "$LOG/ekf.log" 2>&1 &
    nohup python3 "$T/height_slice_viz.py" --ros-args -r __node:=height_slice_viz \
      -p use_sim_time:=false -p floor_z:=-0.35 \
      -p floor_margin:=0.3 -p max_height:=0.3 -p body_exclude_max_x:=0.35 \
      -r cloud_in:=/utlidar/cloud_base_restamped \
      -r cloud_filtered:=/go2_localization/chin_lidar_scan_points > "$LOG/hsv.log" 2>&1 &
    start_slam
    sleep 4
    status
    if ! pgrep -f pointcloud_to_laserscan_node > /dev/null; then
      echo '!! pointcloud_to_laserscan が動いていません(real_up.sh up の状態を確認)'
    fi
    echo "ログ: $LOG/{ekf,hsv,slam}.log"
    ;;
  reset)
    start_slam
    sleep 4
    status
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start|reset|status [ang]"; exit 1
    ;;
esac
