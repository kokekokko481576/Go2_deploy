#!/bin/bash
# 実機ライブで、保存済みの地図の上でAMCL(自己位置推定)を動かす。
#
# 前提: ホストで ./docker/driver/real_up.sh up 済み、かつ live_sept.sh start で9月構成
#       (EKF・床除去 floor_margin 0.3)に切り替え済み。devコンテナ(arbeit-ros2)の中で実行する。
#   docker exec -it arbeit-ros2 bash /home/ros/ros2_ws/bags/tools/live_amcl.sh start
#
#   start [map_yaml] [x] [y] [yaw_rad] : slam_toolboxを止め、地図サーバとAMCLを起動する
#        既定: map_1118e_ang.yaml、初期位置は開始位置のテープ(-0.068, -0.107, -0.325rad)
#   stop   : 地図サーバとAMCLを止める(slam_toolboxは起動し直さない)
#   status : 関係するノードを表示する
#
# slam_toolboxとAMCLはどちらも map->odom を出すので同時には動かせない。start で必ずslamを止める。
# 起動するのは観測系ノードだけで、機体を動かす経路には触らない。
# 注意: killはコンテナ内の同名プロセス全部が対象(ドメインを見ない)。オフライン再生中は使わないこと。
#
# AMCLの上書き設定(2026-11-18、walk_1118e/1118dのオフライン再生で決めた値):
#   robot_model_type : amcl.yamlは差動二輪用(Differential)。Go2は横にも動くので全方向移動用(Omni)にしないと
#                      粒子が全方向に散って発散する(位置ずれ中央値1.1m→0.24m)
#   max_beams        : スキャンは722本中110〜140本しか有効でないので、60本では足りない
#   alpha1-5         : オドメトリの向きは安定しているので動きのノイズを小さくする
#   sigma_hit 等      : スキャンの当てはめ幅を絞る
AMCL_TUNE="-p max_beams:=360 -p robot_model_type:=nav2_amcl::OmniMotionModel \
 -p alpha1:=0.05 -p alpha2:=0.05 -p alpha3:=0.05 -p alpha4:=0.05 -p alpha5:=0.05 \
 -p sigma_hit:=0.1 -p laser_likelihood_max_dist:=0.5"

LOG=/tmp/live_amcl
mkdir -p "$LOG"
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash
CFG=/home/ros/ros2_ws/install/go2_localization/share/go2_localization/config/amcl.yaml

kill_pat() { pgrep -f "$1" | xargs -r kill -9 2>/dev/null; }
stop_amcl() {
  kill_pat 'nav2_lifecycle_manager/lifecycle_manager'
  kill_pat 'nav2_amcl/amcl'
  kill_pat 'nav2_map_server/map_server'
}
status() {
  ps -eo pid,etime,args | grep -E '[e]kf_node|[h]eight_slice_viz|[a]sync_slam_toolbox_node|[p]ointcloud_to_laserscan_node|[n]av2_amcl/amcl|[n]av2_map_server/map_server|[l]ifecycle_manager' | cut -c1-200
}

case "${1:-}" in
  start)
    MAP=${2:-/home/ros/ros2_ws/map_1118e_ang.yaml}; X=${3:--0.068}; Y=${4:--0.107}; YAW=${5:--0.325}
    [ -f "$MAP" ] || { echo "地図が無い: $MAP"; exit 1; }
    pgrep -f 'sept/height_slice_viz.py' > /dev/null || echo '!! 9月構成の床除去が動いていません。先に live_sept.sh start を実行してください'
    # live_sept.sh のあとに real_up.sh up を実行すると既定の床除去が二重に起動し、同じ点群トピックに
    # 2つが配信してしまう(2026-11-19に発生)。9月構成が動いているときは既定の方だけ止める。
    if pgrep -f 'sept/height_slice_viz.py' > /dev/null && pgrep -f 'go2_localization[ /]height_slice_viz' > /dev/null; then
      echo '!! 既定の床除去が二重に動いていたので止めました(9月構成の方だけ残す)'
      kill_pat 'go2_localization[ /]height_slice_viz'
    fi
    kill_pat 'lib/slam_toolbox/'
    stop_amcl
    sleep 1
    nohup /opt/ros/humble/lib/nav2_map_server/map_server --ros-args -r __node:=map_server --params-file $CFG \
      -p use_sim_time:=false -p yaml_filename:=$MAP -r map:=/go2_localization/map > "$LOG/map.log" 2>&1 &
    nohup /opt/ros/humble/lib/nav2_amcl/amcl --ros-args -r __node:=amcl --params-file $CFG -p use_sim_time:=false \
      $AMCL_TUNE -p initial_pose.x:=$X -p initial_pose.y:=$Y -p initial_pose.yaw:=$YAW \
      -r map:=/go2_localization/map -r amcl_pose:=/go2_localization/amcl_pose \
      -r particlecloud:=/go2_localization/particlecloud -r initialpose:=/go2_localization/initialpose > "$LOG/amcl.log" 2>&1 &
    nohup /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_localization \
      --params-file $CFG -p use_sim_time:=false > "$LOG/lcm.log" 2>&1 &
    sleep 6
    status
    grep -q 'Managed nodes are active' "$LOG/lcm.log" && echo 'AMCL: 起動完了(active)' || echo '!! AMCLがactiveになっていません。ログを確認: '"$LOG"'/lcm.log'
    echo "地図: $MAP / 初期位置: x=$X y=$Y yaw=$YAW rad"
    echo "ログ: $LOG/{map,amcl,lcm}.log"
    ;;
  stop)
    stop_amcl; sleep 1; status
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start [map_yaml] [x] [y] [yaw_rad] | stop | status"; exit 1
    ;;
esac
