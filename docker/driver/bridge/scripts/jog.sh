#!/bin/bash
# ブリッジ単体の動作確認。1軸だけを短時間動かして、
# 向き・符号・速さが指令どおりかを目視で確かめる。
#
# 使い方:
#   docker compose exec driver ros2 run go2_sport_bridge jog.sh vx 0.20 1.0
#
#   jog.sh vx 0.20 1.0      # 前進 0.20m/s を1秒
#   jog.sh vy 0.10 1.0      # 左へ 0.10m/s を1秒（正=左。REP-103）
#   jog.sh wz 0.30 1.0      # 左旋回 0.30rad/s を1秒（正=反時計回り）
#
# 事前に: 起立(go2_sport_client 4)＋**機体が「通常モード」であること**、
# そして cmd_vel_to_sport_node を別ターミナルで起動しておくこと。
#
# **このスクリプトは /cmd_vel を直接叩く**（dev側の cmd_vel_safety を経由しない）。
# 速度クランプはブリッジ側の上限しか掛からない点に注意。
#
# **0.15m/s 未満を指定しても機体は進まない。** Go2の歩容はそこが下限で、
# それ未満は胴体が揺れるだけになる(2026-09-02実機実測)。符号確認は 0.20 程度で行う。
set -e
AXIS=${1:?vx|vy|wz を指定}
VAL=${2:?速度}
DUR=${3:-1.0}

# ROSのsetupは未定義変数を参照するので set -u を掛けたまま source しない
set +u
if [ -z "${ROS_DISTRO:-}" ] && [ -f /setup_dds.sh ]; then
    source /setup_dds.sh
fi
set -u

case "$AXIS" in
  vx) TW="{linear: {x: $VAL, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" ;;
  vy) TW="{linear: {x: 0.0, y: $VAL, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" ;;
  wz) TW="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $VAL}}" ;;
  *)  echo "軸は vx|vy|wz"; exit 1 ;;
esac

echo "[jog] $AXIS = $VAL を ${DUR}s。合図から動きます。"
echo "[jog] 止めたいときは Ctrl-C（ブリッジのウォッチドッグが停止指令を出します）"
sleep 1
timeout "$DUR" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "$TW" > /dev/null 2>&1 || true
echo "[jog] 指令を止めました。ウォッチドッグが停止指令を出します。"
