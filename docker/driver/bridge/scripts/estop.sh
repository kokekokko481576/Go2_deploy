#!/bin/bash
# Go2 非常停止。ブリッジを落としてから停止指令を直接送る。
#
# 使い方: driverコンテナで、別ターミナルに常に打てる状態で待機させておく
#   docker compose exec driver ros2 run go2_sport_bridge estop.sh
#
# 順序が重要: 先に指令元を止めないと、止めた直後に次の指令で再び動き出す。
#
# dev側の teleop / cmd_vel_safety は別コンテナなのでここからは落とせないが、
# ブリッジを落とせば cmd_vel は /api/sport/request に変換されなくなるため機体には届かない。
# **ブリッジを落としただけでは止まらない**(機体は最後の指令のまま歩き続ける恐れがある)ので、
# 落としたあとに停止指令を直接送る。
#
# **これはソフト側の停止手段でしかない。** 最後の砦は物理リモコンなので、
# リモコンの操作権を API に渡す設定(UseRemoteCommandFromApi)で走らせないこと。
set -u

echo "[estop] cmd_vel→Moveブリッジを停止します"
pkill -9 -f "lib/go2_sport_bridge/cmd_vel_to_sport_node" 2>/dev/null

# ROSのsetupは未定義変数を参照するので set -u を掛けたまま source しない
set +u
if [ -z "${ROS_DISTRO:-}" ] && [ -f /setup_dds.sh ]; then
    source /setup_dds.sh
fi
set -u

echo "[estop] 停止指令を送ります（ゼロ速度 + StopMove を3秒間）"
# `-w 0` は必須。`ros2 topic pub --once` は既定で購読者が1つ現れるまで待つため、
# 機体が繋がっていない/DDSが見えていない状況では**ここで無限に止まる**。
# 非常停止が黙ってハングするのは最悪なので、待たずに送って3秒間繰り返す
# (繰り返すことで、DDSのマッチング直後に落ちた1通目も取り返せる)。
END=$((SECONDS+3))
while [ $SECONDS -lt $END ]; do
  # ゼロ速度のMove(api_id=1008)。idは毎回変える(同一idの重複指令は機体に無視される)
  ros2 topic pub --once -w 0 /api/sport/request unitree_api/msg/Request \
    "{header: {identity: {id: ${RANDOM}${RANDOM}, api_id: 1008}}, parameter: '{\"x\":0.0,\"y\":0.0,\"z\":0.0}'}" \
    > /dev/null 2>&1
  # StopMove(api_id=1003)
  ros2 topic pub --once -w 0 /api/sport/request unitree_api/msg/Request \
    "{header: {identity: {id: ${RANDOM}${RANDOM}, api_id: 1003}}}" > /dev/null 2>&1
done
echo "[estop] 完了。機体が止まっていることを目視で確認してください。"
echo "[estop] 止まらない場合は Unitree のリモコンで停止してください。"
