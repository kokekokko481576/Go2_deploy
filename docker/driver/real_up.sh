#!/bin/bash
# 実機Go2で自己位置推定/SLAMを動かすための一式を、正しい順序でまとめて起動する。
# (devコンテナだけでなくdriverコンテナ側も含むため dev_up ではなく real_up という名前にした)
#
#   ./docker/driver/real_up.sh            # 観測系のみ起動(機体は動かない)
#   ./docker/driver/real_up.sh --motion   # 走行系(安全フィルタ+cmd_velブリッジ)も起動
#   ./docker/driver/real_up.sh status     # 何が動いているか
#   ./docker/driver/real_up.sh down       # このスクリプトが起動したものを全部止める
#
# 環境変数で上書き可: GO2_NIC(既定 enp2s0) / GO2_IP(既定 192.168.123.161) /
#                     FLOOR_Z(既定 -0.35)
#
# ---------------------------------------------------------------------------
# 起動順と、それぞれの「なぜ」(2026-09-04 実機で判明した事項)
# ---------------------------------------------------------------------------
# 1. NICにIPv4が付いていること。無いとCycloneDDSが
#    `does not match an available interface` で起動できない
# 2. GO2_NIC を driver/dev 双方に渡す。渡し忘れると既定の lo になり、
#    ユニキャスト探索(Peer 127.0.0.1)に切り替わって実機が一切見えない
# 3. 疎通確認は必ず `--no-daemon`。ros2 daemon のキャッシュが、実機が見えていない
#    状態でも121本のトピックを返してくる
# 4. 点群の再スタンプが要る。機体の時計は開発PCより約1110秒遅れており、
#    odom(開発PC時計)と混ざると slam_toolbox が全スキャンを捨てる
#      Message Filter dropping message: ... 'the timestamp on the message is
#      earlier than all the data in the transform cache'
# 5. height_slice_viz の入力は /utlidar/cloud ではなく /utlidar/cloud_base。
#    後者はファームウェアが base_link 座標系で配信するので、**未実測のLiDAR搭載位置TF
#    (sim仮値 pitch=0.35rad)を迂回できる**。floor_z も cafe_world 由来の -0.27 では
#    なく実測 -0.35。この2つを直さないと床を障害物として地図に焼く
#    (実測: 有効ビームの81%が1m未満 -> 0%)
# 6. slam_toolbox は最後。先に上げると床混じりのスキャンで最初の地図が焼き付き、
#    機体が静止している間は更新されない
# ---------------------------------------------------------------------------
set -u

GO2_NIC=${GO2_NIC:-enp2s0}
GO2_IP=${GO2_IP:-192.168.123.161}
FLOOR_Z=${FLOOR_Z:--0.35}
DRIVER=${DRIVER_CONTAINER:-go2-driver}
DEV=${DEV_CONTAINER:-arbeit-ros2}

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
DRIVER_SH='source /setup_dds.sh >/dev/null 2>&1'
DEV_SH='source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash'
CFG=/home/ros/ros2_ws/install/go2_localization/share/go2_localization/config/pointcloud_to_laserscan.yaml

# 起動するプロセスの一覧。停止・状態表示もこの表を使う。
# 形式: <コンテナ> <表示名> <ps照合パターン(実行ファイルのパス片)>
PROCS=(
  "$DRIVER|state_to_odom_imu       |lib/go2_sport_bridge/state_to_odom_imu_node"
  "$DRIVER|cloud_base_restamp      |lib/go2_sport_bridge/utlidar_cloud_restamp_node"
  "$DRIVER|cmd_vel_to_sport(走行系)|lib/go2_sport_bridge/cmd_vel_to_sport_node"
  "$DEV|static_tf               |tf2_ros/static_transform_publisher"
  "$DEV|ekf                     |robot_localization/ekf_node"
  "$DEV|height_slice_viz        |lib/go2_localization/height_slice_viz"
  "$DEV|pointcloud_to_laserscan |pointcloud_to_laserscan/pointcloud_to_laserscan_node"
  "$DEV|slam_toolbox            |slam_toolbox/async_slam_toolbox_node"
  "$DEV|cmd_vel_safety(走行系)   |cmd_vel_safety/cmd_vel_safety_node"
)

die() { echo "[real_up] $*" >&2; exit 1; }
log() { echo "[real_up] $*"; }

# プロセス照合に pgrep -f は使わない。検索文字列を含む自分自身のコマンドラインに
# マッチして「動いている」と誤判定する(pgrep -x はプロセス名が15文字で切られるため
# cmd_vel_to_sport_node のような長い名前で一致しない)。
# ps の結果から、自分の bash -c 自身と grep 自身を除いて数える。
pids_of() {
    local container=$1 pattern=$2
    docker exec "$container" bash -c \
        "ps -eo pid,args | grep -F -- '$pattern' | grep -v ' grep ' | grep -v 'bash -c' | awk '{print \$1}'" \
        2>/dev/null
}

is_running() { [ -n "$(pids_of "$1" "$2")" ]; }

container_up() { docker ps --format '{{.Names}}' | grep -qx "$1"; }

# 起動済みなら何もしない(二重起動すると同名ノードが2つになり、直したのに直らない類の
# 誤診の元になる)
start_node() {
    local container=$1 label=$2 pattern=$3 logfile=$4 cmd=$5
    if is_running "$container" "$pattern"; then
        log "  $label: 既に起動済み(スキップ)"
        return 0
    fi
    docker exec -d "$container" bash -c "$cmd > $logfile 2>&1"
    sleep 2
    if is_running "$container" "$pattern"; then
        log "  $label: 起動 (ログ: $container:$logfile)"
    else
        echo "[real_up]   $label: 起動に失敗。docker exec $container cat $logfile を見ること" >&2
        return 1
    fi
}

cmd_status() {
    local entry container label pattern pids
    for entry in "${PROCS[@]}"; do
        IFS='|' read -r container label pattern <<< "$entry"
        if ! container_up "$container"; then
            printf '  %-24s : コンテナ(%s)が起動していない\n' "$label" "$container"
            continue
        fi
        pids=$(pids_of "$container" "$pattern" | tr '\n' ' ')
        if [ -n "${pids// /}" ]; then
            printf '  %-24s : 稼働中  pid=%s\n' "$label" "${pids% }"
        else
            printf '  %-24s : 停止\n' "$label"
        fi
    done
}

cmd_down() {
    local entry container label pattern pids
    # 指令源(走行系)を先に落とす。estop.sh と同じ順序。
    for entry in "${PROCS[@]}"; do
        IFS='|' read -r container label pattern <<< "$entry"
        container_up "$container" || continue
        pids=$(pids_of "$container" "$pattern" | tr '\n' ' ')
        [ -n "${pids// /}" ] || continue
        docker exec "$container" bash -c "kill -9 ${pids} 2>/dev/null; true"
        log "停止: $label (pid ${pids% })"
    done
    sleep 2
    log "--- 停止後の状態 ---"
    cmd_status
    log "機体が動いていないことを目視で確認すること。止まらない場合はリモコンで停止。"
}

check_preconditions() {
    log "0. 前提確認"

    ip -brief link show "$GO2_NIC" >/dev/null 2>&1 \
        || die "NIC $GO2_NIC が無い。ip -brief link で名前を確認すること"

    ip -4 addr show "$GO2_NIC" | grep -q 'inet ' \
        || die "$GO2_NIC にIPv4が付いていない。CycloneDDSはIPv4未割当のNICを使えない
         sudo ip addr add 192.168.123.99/24 dev $GO2_NIC"
    log "  NIC $GO2_NIC : $(ip -4 -brief addr show "$GO2_NIC" | awk '{print $3}')"

    ping -c 2 -W 2 "$GO2_IP" >/dev/null 2>&1 \
        || die "$GO2_IP にpingが通らない。LANケーブルと機体の電源を確認すること"
    log "  ping $GO2_IP : OK"
}

start_containers() {
    log "1. コンテナ起動 (GO2_NIC=$GO2_NIC)"
    if container_up "$DRIVER"; then
        local nic
        nic=$(docker inspect "$DRIVER" --format '{{range .Config.Env}}{{println .}}{{end}}' \
              | grep '^GO2_NIC=' | cut -d= -f2)
        [ "$nic" = "$GO2_NIC" ] || die "driverコンテナが GO2_NIC=$nic で動いている(欲しいのは $GO2_NIC)。
         作り直すこと: cd docker/driver && docker compose down && GO2_NIC=$GO2_NIC docker compose up -d
         (--force-recreate では古いコンテナが残ることがある)"
        log "  driver: 起動済み (GO2_NIC=$nic)"
    else
        (cd "$REPO_ROOT/docker/driver" && GO2_NIC=$GO2_NIC docker compose up -d >/dev/null) \
            || die "driverコンテナの起動に失敗"
        log "  driver: 起動"
    fi

    if container_up "$DEV"; then
        log "  dev: 起動済み"
    else
        (cd "$REPO_ROOT/docker" && GO2_NIC=$GO2_NIC docker compose up -d >/dev/null) \
            || die "devコンテナの起動に失敗"
        log "  dev: 起動"
    fi
    sleep 3
}

check_dds() {
    log "2. 実機トピックの疎通確認 (--no-daemon)"
    local n
    n=$(docker exec "$DRIVER" bash -c \
        "$DRIVER_SH; ros2 topic list --no-daemon 2>/dev/null | wc -l")
    # loで起動していると /parameter_events と /rosout の2本しか出ない
    [ "${n:-0}" -gt 10 ] \
        || die "driverから実機トピックが見えない(${n}本)。GO2_NICの指定を確認すること"
    log "  driver: ${n}本"

    n=$(docker exec "$DEV" bash -c \
        "source /opt/ros/humble/setup.bash; ros2 topic list --no-daemon 2>/dev/null | wc -l")
    [ "${n:-0}" -gt 10 ] || die "devから実機トピックが見えない(${n}本)"
    log "  dev: ${n}本"
}

start_driver_nodes() {
    log "3. driver側"
    start_node "$DRIVER" "state_to_odom_imu" "lib/go2_sport_bridge/state_to_odom_imu_node" \
        /tmp/odom.log \
        "$DRIVER_SH; exec ros2 run go2_sport_bridge state_to_odom_imu_node" || return 1
    start_node "$DRIVER" "cloud_base_restamp" "lib/go2_sport_bridge/utlidar_cloud_restamp_node" \
        /tmp/restamp_base.log \
        "$DRIVER_SH; exec ros2 run go2_sport_bridge utlidar_cloud_restamp_node --ros-args \
             -r __node:=utlidar_cloud_base_restamp_node \
             -r cloud_in:=/utlidar/cloud_base -r cloud_out:=/utlidar/cloud_base_restamped" || return 1
    sleep 2
    local off
    off=$(docker exec "$DRIVER" bash -c "grep -o '機体クロックとの差: [^ /]*' /tmp/restamp_base.log | tail -1")
    [ -n "$off" ] && log "  $off (この分だけstampを進めて中継している)"
}

start_dev_chain() {
    log "4. dev側の推定チェーン"
    start_node "$DEV" "static_tf" "tf2_ros/static_transform_publisher" /tmp/tf.log \
        "$DEV_SH; exec ros2 launch go2_localization static_tf_real.launch.py" || return 1
    start_node "$DEV" "ekf" "robot_localization/ekf_node" /tmp/ekf.log \
        "$DEV_SH; exec ros2 launch go2_localization ekf_real.launch.py" || return 1
    start_node "$DEV" "pointcloud_to_laserscan" \
        "pointcloud_to_laserscan/pointcloud_to_laserscan_node" /tmp/p2l.log \
        "$DEV_SH; exec ros2 launch go2_localization pointcloud_to_laserscan_real.launch.py" || return 1

    # height_slice_viz だけは launch を使わず手動。mapping_real.launch.py の中では
    # cloud_in が /utlidar/cloud に直結されていて、上記5の理由で床除去が壊れるため
    start_node "$DEV" "height_slice_viz" "lib/go2_localization/height_slice_viz" /tmp/hsv.log \
        "$DEV_SH; exec ros2 run go2_localization height_slice_viz --ros-args \
             -r __node:=height_slice_viz --params-file $CFG \
             -p use_sim_time:=false -p floor_z:=$FLOOR_Z \
             -r cloud_in:=/utlidar/cloud_base_restamped \
             -r cloud_filtered:=/go2_localization/chin_lidar_scan_points" || return 1
}

# 床除去が効いているかの自己検定。これが崩れると地図に床が焼き付くが、ログには
# 何も出ないので気づけない。slam_toolbox を上げる前に一度だけ確かめる。
check_floor_removal() {
    log "5. 床除去の自己検定"
    docker exec "$DEV" bash -c "$DEV_SH; cat > /tmp/floor_check.py <<'PYEOF'
import math, sys, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class C(Node):
    def __init__(self):
        super().__init__('floor_check')
        self.create_subscription(LaserScan, '/go2_localization/chin_lidar_scan',
                                 self.cb, rclpy.qos.qos_profile_sensor_data)
        self.done = False
    def cb(self, m):
        if self.done:
            return
        self.done = True
        r = [x for x in m.ranges if not math.isinf(x) and not math.isnan(x)]
        if not r:
            print('NG 有効ビームが0本。点群が届いていない')
            sys.exit(1)
        near = sum(1 for x in r if x < 1.0)
        pct = 100.0 * near / len(r)
        r.sort()
        print(f'有効{len(r)}本 中央値{r[len(r)//2]:.2f}m 1m未満{pct:.1f}%')
        # 床を拾っていると1m未満が支配的になる(実測: 修正前81.3% -> 修正後0.0%)
        sys.exit(1 if pct > 30.0 else 0)

rclpy.init()
c = C()
while rclpy.ok() and not c.done:
    rclpy.spin_once(c, timeout_sec=1.0)
PYEOF
true"
    local out rc
    out=$(docker exec "$DEV" bash -c "$DEV_SH; timeout 25 python3 /tmp/floor_check.py" 2>/dev/null)
    rc=$?
    log "  $out"
    if [ "$rc" -ne 0 ]; then
        echo "[real_up]   1m未満のビームが多すぎる。床を障害物として拾っている疑い。" >&2
        echo "[real_up]   FLOOR_Z(現在 $FLOOR_Z)を機体の実際の立ち高さに合わせること。" >&2
        echo "[real_up]   目安: ros2 topic echo --once /go2_state_bridge/odom の position.z の符号反転" >&2
        return 1
    fi
}

start_slam() {
    log "6. slam_toolbox (最後に起動する)"
    start_node "$DEV" "slam_toolbox" "slam_toolbox/async_slam_toolbox_node" /tmp/slam.log \
        "$DEV_SH; exec ros2 launch go2_localization slam_real.launch.py" || return 1
    sleep 8
    if docker exec "$DEV" grep -q "earlier than all the data in the transform cache" /tmp/slam.log 2>/dev/null; then
        echo "[real_up]   スキャンが時刻ずれで捨てられている。再スタンプ中継が効いていない" >&2
        return 1
    fi
    log "  スキャンの取りこぼし無し"
}

start_motion() {
    log "7. 走行系"
    echo "[real_up]   ** 機体が動く経路を有効にする。周囲の安全と、estop.shを打てる別ターミナルを確保すること **"
    echo "[real_up]   ** Unitree Goアプリで「通常モード」にしておくこと(AIモードだと脚が出ない) **"
    start_node "$DEV" "cmd_vel_safety" "cmd_vel_safety/cmd_vel_safety_node" /tmp/safety.log \
        "$DEV_SH; exec ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args \
             -p max_linear_x:=0.22 -p max_linear_y:=0.18 -p max_angular_z:=0.45" || return 1
    start_node "$DRIVER" "cmd_vel_to_sport" "lib/go2_sport_bridge/cmd_vel_to_sport_node" \
        /tmp/bridge.log \
        "$DRIVER_SH; exec ros2 run go2_sport_bridge cmd_vel_to_sport_node" || return 1
}

cmd_up() {
    local with_motion=$1
    check_preconditions
    start_containers
    check_dds
    start_driver_nodes || die "driver側の起動に失敗"
    start_dev_chain    || die "dev側の起動に失敗"
    check_floor_removal || die "床除去の検定に失敗。この状態でslam_toolboxを上げると地図に床が焼き付く"
    start_slam         || die "slam_toolboxの起動に失敗"
    [ "$with_motion" = yes ] && { start_motion || die "走行系の起動に失敗"; }

    echo
    log "--- 起動完了 ---"
    cmd_status
    echo
    cat <<MSG
[real_up] 次にやること:
[real_up]   地図を見る : docker exec -it $DEV bash -c 'source /opt/ros/humble/setup.bash; rviz2'
[real_up]                (Fixed Frame: map / トピック /go2_localization/map)
MSG
    if [ "$with_motion" = yes ]; then
        cat <<MSG
[real_up]   非常停止   : docker exec -it $DRIVER bash -c 'source /setup_dds.sh; ros2 run go2_sport_bridge estop.sh'
[real_up]                (別ターミナルで打てる状態にしてから走らせる。最後の砦は物理リモコン)
[real_up]   キーボード : ./docker/driver/teleop_real.sh
[real_up]                前進(vx)と旋回(wz)のみ使う。横移動(vy)は効かない上に前進を殺す
MSG
    else
        cat <<MSG
[real_up]   走らせる   : リモコンで歩かせればそのまま地図が育つ。
[real_up]                キーボードで操作したいなら --motion を付けて起動し直すこと
MSG
    fi
    cat <<MSG
[real_up]   地図の保存 : docker exec -it $DEV bash -c 'source /opt/ros/humble/setup.bash; \\
[real_up]                  ros2 run nav2_map_server map_saver_cli -f /home/ros/ros2_ws/map_real_01 \\
[real_up]                  --ros-args -p save_map_timeout:=5.0 -r map:=/go2_localization/map'
[real_up]   停止       : ./docker/driver/real_up.sh down
MSG
}

WITH_MOTION=no
ACTION=up
for arg in "$@"; do
    case "$arg" in
        up)        ACTION=up ;;
        down)      ACTION=down ;;
        status)    ACTION=status ;;
        --motion)  WITH_MOTION=yes ;;
        -h|--help) sed -n '2,14p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *)         die "不明な引数: $arg (up|down|status|--motion)" ;;
    esac
done

case "$ACTION" in
    up)     cmd_up "$WITH_MOTION" ;;
    down)   cmd_down ;;
    status) cmd_status ;;
esac
