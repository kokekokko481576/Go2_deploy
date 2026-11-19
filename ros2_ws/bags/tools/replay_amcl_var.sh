#!/bin/bash
# usage: [DOMAIN=46] [PLAY_TIMEOUT=秒] [AMCL_EXTRA="-p max_beams:=360"] replay_amcl_var.sh <name> <rate> <bag> <map_yaml> <x> <y> <yaw>
# 9月構成(床除去floor_margin 0.3)で生データからスキャンを作り直し、既存地図の上でAMCLを回す。
# ライブ(既定ドメイン)と並走可。odom->base_linkのTFはEKF自身が出す(odom_tf.pyは使わない。
# 以前の再生はEKFとodom_tf.pyが同じTFを二重に出していた)。結果は <name>.amcl.tsv
NAME=$1; RATE=$2; BAG=$3; MAP=$4; X=$5; Y=$6; YAW=$7
export ROS_DOMAIN_ID=${DOMAIN:-46} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
T=/home/ros/ros2_ws/bags/tools
SHARE=~/ros2_ws/install/go2_localization/share/go2_localization/config
OUT=/home/ros/ros2_ws/bags/replay; mkdir -p $OUT
P=()
/opt/ros/humble/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node \
  --params-file $T/sept/ekf.yaml -p use_sim_time:=true \
  -r raw_odom_input:=/go2_state_bridge/odom -r imu_plugin/out:=/go2_state_bridge/imu \
  -r odometry/filtered:=/sept/odometry/filtered > $OUT/$NAME.ekf.log 2>&1 & P+=($!)
python3 $T/sept/height_slice_viz.py --ros-args -r __node:=height_slice_viz -p use_sim_time:=true \
  -p floor_z:=-0.35 -p floor_margin:=0.3 -p max_height:=0.3 -p body_exclude_max_x:=0.35 \
  -r cloud_in:=/utlidar/cloud_base_restamped -r cloud_filtered:=/sept/points > $OUT/$NAME.hsv.log 2>&1 & P+=($!)
/opt/ros/humble/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node --ros-args \
  -r __node:=pointcloud_to_laserscan --params-file $SHARE/pointcloud_to_laserscan.yaml \
  -p use_sim_time:=true -r cloud_in:=/sept/points -r scan:=/sept/scan > $OUT/$NAME.p2l.log 2>&1 & P+=($!)
/opt/ros/humble/lib/nav2_map_server/map_server --ros-args -r __node:=map_server --params-file $SHARE/amcl.yaml \
  -p use_sim_time:=true -p yaml_filename:=$MAP -r map:=/go2_localization/map > $OUT/$NAME.map.log 2>&1 & P+=($!)
/opt/ros/humble/lib/nav2_amcl/amcl --ros-args -r __node:=amcl --params-file $SHARE/amcl.yaml -p use_sim_time:=true \
  -p scan_topic:=/sept/scan ${AMCL_EXTRA:-} -p initial_pose.x:=$X -p initial_pose.y:=$Y -p initial_pose.yaw:=$YAW \
  -r map:=/go2_localization/map -r amcl_pose:=/go2_localization/amcl_pose \
  -r particlecloud:=/go2_localization/particlecloud -r initialpose:=/go2_localization/initialpose > $OUT/$NAME.amcl.log 2>&1 & P+=($!)
/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_localization \
  --params-file $SHARE/amcl.yaml -p use_sim_time:=true > $OUT/$NAME.lcm.log 2>&1 & P+=($!)
python3 $T/amcl_pose_logger.py $OUT/$NAME.amcl.tsv --ros-args -p use_sim_time:=true > $OUT/$NAME.rec.log 2>&1 & P+=($!)
sleep 6
timeout ${PLAY_TIMEOUT:-100000} ros2 bag play $BAG --clock 100 -r $RATE \
  --topics /go2_state_bridge/odom /utlidar/cloud_base_restamped /go2_state_bridge/imu /tf_static > $OUT/$NAME.play.log 2>&1
sleep 3
kill "${P[@]}"; wait 2>/dev/null
echo "$NAME: amcl_pose $(wc -l < $OUT/$NAME.amcl.tsv)行"
