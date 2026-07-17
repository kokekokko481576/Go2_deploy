#!/usr/bin/env bash
# 自作AMCLに初期姿勢を与え直す(Issue #27)。
# teleop等でロボットを動かした後にlocalizationを起動してしまい、
# 「map原点にいる」前提が外れて自己位置が破綻したときの復旧手段その1。
#
# 使い方(devコンテナ内、localization起動中に):
#   ./set_initial_pose.sh              # 原点(0,0,0度)に戻す
#   ./set_initial_pose.sh 1.5 0.5 90   # x=1.5 y=0.5 yaw=90度
#
# 実際のロボット位置(map座標)は Gazebo の画面か、upstream側が動いていれば
#   ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot1/tf
# で読める。
set -eu

X="${1:-0.0}"
Y="${2:-0.0}"
YAW_DEG="${3:-0.0}"

for v in "$X" "$Y" "$YAW_DEG"; do
  if ! [[ "$v" =~ ^[+-]?[0-9]+(\.[0-9]+)?$ ]]; then
    echo "error: '$v' は数値ではありません。使い方: set_initial_pose.sh [x] [y] [yaw度]" >&2
    exit 1
  fi
done

read -r QZ QW < <(python3 -c "import math, sys
r = float(sys.argv[1]) * math.pi / 180.0 / 2.0
print(f'{math.sin(r):.6f} {math.cos(r):.6f}')" "$YAW_DEG") || {
  echo "error: yaw->quaternion変換に失敗しました" >&2; exit 1; }

echo "publish /go2_localization/initialpose: x=${X} y=${Y} yaw=${YAW_DEG}deg"
# 共分散はRVizの2D Pose Estimateと同じ既定値(xy 0.25, yaw 0.068)。
# --onceはsubscriberマッチング前にpublishして落とすことがあるため、
# -w 1(マッチ待ち)+複数回送信にしている(混在環境の配送遅延対策、#7周辺症状)
exec ros2 topic pub /go2_localization/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: ${X}, y: ${Y}}, orientation: {z: ${QZ}, w: ${QW}}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}" \
  -w 1 -t 3 -r 1
