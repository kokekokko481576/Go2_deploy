#!/bin/bash
# ブリッジ単体の動作確認。1軸だけを短時間動かして、
# 向き・符号・速さが指令どおりかを目視で確かめる。
#
# 使い方:
#   docker compose exec driver ros2 run go2_sport_bridge jog.sh vx 0.40 1.0
#
#   jog.sh vx 0.40 1.0      # 前進 0.40m/s を1秒
#   jog.sh vy 0.10 1.0      # 左へ 0.10m/s を1秒（正=左。REP-103）
#   jog.sh wz 0.60 1.0      # 左旋回 0.60rad/s を1秒（正=反時計回り）
#
# 事前に: 起立(go2_sport_client 4)＋**機体が「通常モード」であること**、
# そして cmd_vel_to_sport_node を別ターミナルで起動しておくこと。
#
# 出力先は起動時に決める。dev側の cmd_vel_safety が動いていれば /cmd_vel_raw に送り、
# 安全フィルタを経由させる。/cmd_vel に直接送ると、cmd_vel_safety が指令の途絶中に
# 20Hzで出しているゼロ速度と交互に混ざり、機体には実質半分の指令しか届かない
# (「jogを打っても進まない」に見える)。
# 動いていなければ /cmd_vel を直接叩く(ブリッジ単体の確認。クランプはブリッジ側の上限のみ)。
#
# **前進は0.20m/s以下だと足がほとんど出ない**(実動率34%)。0.30以下は直進性も崩れ、
# 実用下限は0.35(2026-09-23実機実測、#75)。符号確認は vx 0.40 / wz 0.60 程度で行う。
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

# ros2 daemon のキャッシュは実際に居ないノードを返すことがあるので --no-daemon で数える
SUBS=$(ros2 topic info /cmd_vel_raw --no-daemon --spin-time 2 2>/dev/null \
    | awk '/^Subscription count:/ {print $3}')
if [ "${SUBS:-0}" -ge 1 ]; then
    TOPIC=/cmd_vel_raw
    echo "[jog] cmd_vel_safety が動いているので /cmd_vel_raw に送る(安全フィルタの上限が掛かる)"
else
    TOPIC=/cmd_vel
    echo "[jog] cmd_vel_safety が見えないので /cmd_vel に直接送る(上限はブリッジ側のみ)"
fi

echo "[jog] $AXIS = $VAL を ${DUR}s。合図から動きます。"
echo "[jog] 止めたいときは Ctrl-C（ブリッジのウォッチドッグが停止指令を出します）"
sleep 1
timeout "$DUR" ros2 topic pub -r 20 "$TOPIC" geometry_msgs/msg/Twist "$TW" > /dev/null 2>&1 || true
echo "[jog] 指令を止めました。ウォッチドッグが停止指令を出します。"
