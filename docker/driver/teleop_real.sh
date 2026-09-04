#!/bin/bash
# 実機Go2をキーボードで走らせる(sim版 docker/sim/teleop.sh の実機版)。
#
# sim版との違い:
#   - sim版はgo2-simコンテナ(Jazzy)の中で /robot1/cmd_vel へ流す。実機には
#     /robot1 名前空間が無く、コンテナもdev(arbeit-ros2/Humble)側になる
#   - 実機では cmd_vel_safety を必ず経由させる。teleopは直接 cmd_vel を叩かず
#     cmd_vel_raw に出し、速度・加速度の上限とウォッチドッグを挟んでから機体に届ける
#
# 実機での指令の流れ:
#   teleop_twist_keyboard(dev) --cmd_vel_raw--> cmd_vel_safety(dev)
#     --cmd_vel--> cmd_vel_to_sport_node(driver) --/api/sport/request--> 機体
#
# 事前に必要なもの(このスクリプトが起動前に確認する):
#   1. driverコンテナで cmd_vel_to_sport_node が起動していること
#   2. devコンテナで cmd_vel_safety_node が起動していること
#   3. **機体がUnitree Goアプリで「通常モード」になっていること**(このスクリプトからは
#      確認できない。AIモードのままだとMoveは受理されるのに脚が出ない)
#
# 非常停止は別ターミナルで打てる状態にしておくこと:
#   docker exec -it go2-driver bash -c 'source /setup_dds.sh; ros2 run go2_sport_bridge estop.sh'
# **最後の砦は物理リモコン。**
set -u

DRIVER_CONTAINER=${DRIVER_CONTAINER:-go2-driver}
DEV_CONTAINER=${DEV_CONTAINER:-arbeit-ros2}

# プロセス確認に pgrep -f は使わない。検索文字列を含む自分自身のコマンドラインに
# マッチして「動いている」と誤判定する。pgrep -x もプロセス名が15文字で切られるため
# cmd_vel_to_sport_node のような長い名前では一致しない。
# ps + grep -v grep で数える。
running_in() {
    local container=$1 pattern=$2
    local n
    n=$(docker exec "$container" bash -c \
        "ps -ef | grep -F '$pattern' | grep -v grep | wc -l" 2>/dev/null) || return 1
    [ "${n:-0}" -ge 1 ]
}

fail=0

if ! docker ps --format '{{.Names}}' | grep -qx "$DRIVER_CONTAINER"; then
    echo "[teleop] driverコンテナ($DRIVER_CONTAINER)が起動していない。" >&2
    echo "         cd docker/driver && GO2_NIC=<実機のNIC名> docker compose up -d" >&2
    fail=1
elif ! running_in "$DRIVER_CONTAINER" "lib/go2_sport_bridge/cmd_vel_to_sport_node"; then
    echo "[teleop] cmd_vel_to_sport_node が動いていない。cmd_velは機体に届かない。" >&2
    echo "         docker exec -d $DRIVER_CONTAINER bash -c 'source /setup_dds.sh; \\" >&2
    echo "           exec ros2 run go2_sport_bridge cmd_vel_to_sport_node'" >&2
    fail=1
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$DEV_CONTAINER"; then
    echo "[teleop] devコンテナ($DEV_CONTAINER)が起動していない。" >&2
    echo "         cd docker && GO2_NIC=<実機のNIC名> docker compose up -d" >&2
    fail=1
elif ! running_in "$DEV_CONTAINER" "cmd_vel_safety/cmd_vel_safety_node"; then
    echo "[teleop] cmd_vel_safety_node が動いていない。安全フィルタ無しで走らせないこと。" >&2
    echo "         実機で詰めた上限値(2026-09-02)で起動する:" >&2
    echo "         docker exec -d $DEV_CONTAINER bash -c 'source /opt/ros/humble/setup.bash; \\" >&2
    echo "           source ~/ros2_ws/install/setup.bash; \\" >&2
    echo "           exec ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args \\" >&2
    echo "             -p max_linear_x:=0.22 -p max_linear_y:=0.18 -p max_angular_z:=0.45'" >&2
    fail=1
fi

[ "$fail" -eq 0 ] || exit 1

cat <<'MSG'
[teleop] 前提OK。キーボードで走ります。
[teleop]   - 機体が「通常モード」でないと、指令は通っても脚が出ません(胴体だけ揺れる)
[teleop]   - 横移動(vy)はGo2ではほとんど効きません。前進(vx)と旋回(wz)で操作してください
[teleop]   - 0.15m/s未満は歩容の下限を割るため進みません
[teleop]   - キーを離す(何も送らない)と0.5秒でウォッチドッグが停止させます
MSG

exec docker exec -it "$DEV_CONTAINER" bash -c \
    '. /opt/ros/humble/setup.bash && . "$HOME/ros2_ws/install/setup.bash" && \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r /cmd_vel:=/cmd_vel_raw'
