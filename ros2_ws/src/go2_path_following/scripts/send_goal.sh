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

# typo対策(例: "3.0.0.0" スペースのつもりがドット)。数値以外は即エラーで止める
for v in "$X" "$Y" "$YAW_DEG"; do
  if ! [[ "$v" =~ ^[+-]?[0-9]+(\.[0-9]+)?$ ]]; then
    echo "error: '$v' は数値ではありません。使い方: send_goal.sh [x] [y] [yaw度] (スペース区切り)" >&2
    exit 1
  fi
done

# yaw(度)→クォータニオン(z,w)。roll/pitchは0固定
# (awkはdevコンテナに入っていないためpython3を使う。ROS2環境なら必ず存在する)
read -r QZ QW < <(python3 -c "import math, sys
r = float(sys.argv[1]) * math.pi / 180.0 / 2.0
print(f'{math.sin(r):.6f} {math.cos(r):.6f}')" "$YAW_DEG") || {
  echo "error: yaw->quaternion変換に失敗しました" >&2; exit 1; }

echo "publish /goal_pose: x=${X} y=${Y} yaw=${YAW_DEG}deg (qz=${QZ} qw=${QW})"
# --onceはsubscriberマッチング前にpublishして落とすことがあるため、
# -w 1(マッチ待ち)+複数回送信にしている(set_initial_pose.shと同じ対策)。
# Humble⇔Jazzy混在環境では配送が遅延する場合があるため3回に増やしてある(#7周辺症状)
exec ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: ${X}, y: ${Y}}, orientation: {z: ${QZ}, w: ${QW}}}}" \
  -w 1 -t 3 -r 1
