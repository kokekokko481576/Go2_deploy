#!/usr/bin/env bash
# Go2_deploy: GATE1計測用ゴール投入スクリプト (Issue #36)
# 使い方: ./scripts/send_goal.sh [x] [y] [yaw(度)]
# 例: ./scripts/send_goal.sh 3.0 0.0 90

set -u

X="${1:-3.0}"
Y="${2:-0.0}"
YAW_DEG="${3:-0}"

YAW_RAD=$(python3 -c "import math; print(math.radians($YAW_DEG))")
HALF_YAW=$(python3 -c "import math; print($YAW_RAD / 2)")
QZ=$(python3 -c "import math; print(math.sin($HALF_YAW))")
QW=$(python3 -c "import math; print(math.cos($HALF_YAW))")

echo "Sending goal: x=$X, y=$Y, yaw=$YAW_DEG°"

ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}" \
  -w 1 -t 3 -r 1 2>/dev/null

echo "Goal sent."
