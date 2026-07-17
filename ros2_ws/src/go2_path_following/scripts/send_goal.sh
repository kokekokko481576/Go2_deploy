#!/usr/bin/env bash
# /goal_pose にPoseStampedを1回publishする(GATE1動作確認用、Issue #35)。
# 長いYAMLを手打ちするとスペース位置を間違えやすいのでスクリプト化した。
#
# 使い方(devコンテナ内):
#   ./send_goal.sh              # 既定: x=3.0 y=0.0 yaw=0度
#   ./send_goal.sh 2.0 1.5      # x=2.0 y=1.5 yaw=0度
#   ./send_goal.sh 2.0 1.5 90   # x=2.0 y=1.5 yaw=90度(左向き)
set -eu

X="${1:-3.0}"
Y="${2:-0.0}"
YAW_DEG="${3:-0.0}"

# yaw(度)→クォータニオン(z,w)。roll/pitchは0固定
read -r QZ QW < <(awk -v d="$YAW_DEG" \
  'BEGIN { r = d * 3.14159265358979 / 180 / 2; printf "%.6f %.6f", sin(r), cos(r) }')

echo "publish /goal_pose: x=${X} y=${Y} yaw=${YAW_DEG}deg (qz=${QZ} qw=${QW})"
exec ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: ${X}, y: ${Y}}, orientation: {z: ${QZ}, w: ${QW}}}}" \
  --once
