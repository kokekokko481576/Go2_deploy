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
# 照合文字列は real_up.sh(PROCS) と teleop_real.sh(running_in) にもある。
# リネームで1箇所だけずれると黙って0件になるので、0件のときは必ず表示する
if pkill -9 -f "lib/go2_sport_bridge/cmd_vel_to_sport_node" 2>/dev/null; then
    echo "[estop] ブリッジを停止した"
else
    echo "[estop] 停止するブリッジが見つからなかった(起動していないか、照合文字列がずれている)" >&2
fi

# ROSのsetupは未定義変数を参照するので set -u を掛けたまま source しない
set +u
if [ -z "${ROS_DISTRO:-}" ] && [ -f /setup_dds.sh ]; then
    source /setup_dds.sh
fi
set -u

echo "[estop] 停止指令を送ります（ゼロ速度Move を20Hz・StopMove を1秒ごと、機体が見えてから3秒間）"
# `ros2 topic pub --once` を繰り返す形にしない。publisher がDDSのマッチング前に
# 破棄されるとメッセージはどこにも届かない(実LANのマッチングは数百ms〜秒かかる)。
# estop_send_node は1プロセスでpublisherを生かしたまま送り続け、
# 機体が見えないままでも6秒で打ち切る(黙ってハングしない)。
ros2 run go2_sport_bridge estop_send_node
rc=$?
echo "[estop] 完了。機体が止まっていることを目視で確認してください。"
echo "[estop] 止まらない場合は Unitree のリモコンで停止してください。"
exit $rc
