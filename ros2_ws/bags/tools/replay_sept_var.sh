#!/bin/bash
# usage: replay_sept_var.sh <name> <rate> <bag_path> [slam_toolboxへの追加引数 例: -p coarse_search_angle_offset:=0.1]
# replay_sept_full.shの派生。ライブ(既定ドメイン)が動いていても並行で回せるよう、
# 設定はtools/sept/から読み、SLAM本数チェックはせず、自分が起動したPIDだけを止める。
# map->odomの向きの時系列を <name>.yaw.csv に残す。
NAME=$1; RATE=$2; BAG=$3
export ROS_DOMAIN_ID=${DOMAIN:-46} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
T=/home/ros/ros2_ws/bags/tools
SHARE=~/ros2_ws/install/go2_localization/share/go2_localization/config
OUT=/home/ros/ros2_ws/bags/replay; mkdir -p $OUT
/opt/ros/humble/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node \
  --params-file $T/sept/ekf.yaml -p use_sim_time:=true \
  -r raw_odom_input:=/go2_state_bridge/odom -r imu_plugin/out:=/go2_state_bridge/imu \
  -r odometry/filtered:=/sept/odometry/filtered > $OUT/$NAME.ekf.log 2>&1 &
P1=$!
python3 $T/sept/height_slice_viz.py --ros-args -r __node:=height_slice_viz -p use_sim_time:=true \
  -p floor_z:=-0.35 $HSV_ARGS -r cloud_in:=/utlidar/cloud_base_restamped -r cloud_filtered:=/sept/points > $OUT/$NAME.hsv.log 2>&1 &
P2=$!
/opt/ros/humble/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node --ros-args \
  -r __node:=pointcloud_to_laserscan --params-file $SHARE/pointcloud_to_laserscan.yaml \
  -p use_sim_time:=true -r cloud_in:=/sept/points -r scan:=/sept/scan > $OUT/$NAME.p2l.log 2>&1 &
P3=$!
python3 $T/odom_tf.py --ros-args -p use_sim_time:=true -p odom_topic:=/sept/odometry/filtered > $OUT/$NAME.tf.log 2>&1 &
P4=$!
/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node --ros-args \
  --params-file $T/sept/slam_toolbox_real.yaml -p use_sim_time:=true -p scan_topic:=/sept/scan \
  "${@:4}" > $OUT/$NAME.slam.log 2>&1 &
P5=$!
python3 $T/yaw_logger.py $OUT/$NAME.yaw.csv > /dev/null 2>&1 &
P6=$!
python3 $T/pose_logger.py $OUT/$NAME.pose.tsv --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
P7=$!
sleep 4
timeout ${PLAY_TIMEOUT:-100000} ros2 bag play $BAG --clock 100 -r $RATE \
  --topics /go2_state_bridge/odom /utlidar/cloud_base_restamped /go2_state_bridge/imu /tf_static > $OUT/$NAME.play.log 2>&1
sleep 5
ros2 run nav2_map_server map_saver_cli -f $OUT/$NAME --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 > $OUT/$NAME.save.log 2>&1
kill $P1 $P2 $P3 $P4 $P5 $P6 $P7; wait 2>/dev/null
grep -c "Map saved successfully" $OUT/$NAME.save.log
python3 - $OUT/$NAME.yaw.csv <<'PY'
import sys
r=[l.split(',') for l in open(sys.argv[1]) if l.strip()]
y=[float(a[1]) for a in r]
w=lambda a:(a+180)%360-180
j=[w(y[k]-y[k-1]) for k in range(1,len(y)) if abs(w(y[k]-y[k-1]))>8]
print('map->odom更新 %d回, 8度超の飛び %d回, 最終の向き %+.0f度, 範囲 %+.0f〜%+.0f'%(len(y),len(j),y[-1] if y else 0,min(y) if y else 0,max(y) if y else 0))
PY
