#!/usr/bin/env bash
# Go2_deploy: 日常の起動オーケストレータ (Issue #42)
# リポジトリ直下で: ./scripts/dev-up.sh [--gate1] [--no-build]
#
# やること: sim/dev コンテナ起動 → ロボット起動待ち → colcon build →
#   対話で選んだ自作スタック(自己位置推定/経路生成/経路追従)を1ターミナルに
#   ログ集約して前景起動。起動後は別ターミナルで自分の新規ノードを動かすだけ。
#
# 初回(イメージ未ビルド)は先に ./scripts/first-run.sh を推奨(ビルド+疎通確認)。
set -u
cd "$(dirname "$0")/.."

SIM_CONTAINER=go2-sim
DEV_CONTAINER=arbeit-ros2
DEMO_LAUNCH='~/ros2_ws/launch/demo.launch.py'   # コンテナ内パス(ros2_ws はマウント済み)

GATE1=0
SIM_NAV2=0        # 既定: sim 付属の upstream Nav2 は起動しない(自作スタックと冗長で重い)
DO_BUILD=1

show_help() {
    cat <<'HELP'
使い方: ./scripts/dev-up.sh [オプション]

  docker(sim/dev) の起動 → colcon build → 対話で選んだ自作スタックを前景起動。

  sim は既定で「本家(upstream)Nav2 なし」で起動する。本プロジェクトは自作の
  自己位置推定/経路追従を動かすため本家Nav2は冗長で重く、/robot1/cmd_vel の
  二重publisher原因にもなるため(#35)。

オプション:
  --sim-nav2  sim を本家 Nav2 付きで起動する(#5 の本家スタック比較用。重い)。
  --gate1     GATE1計測モード。本家Nav2オフに加え可視化(専用RViz)も止める(最軽量・自作スタックだけで駆動)。
  --no-build  colcon build を飛ばす(直前のビルドをそのまま使う)。
  -h, --help  このヘルプ。

対話で聞くこと:
  1) 自己位置推定 … 実装済み(EKF/AMCL) か 自作(自分で /go2_localization/tf を出す)
  2) 経路生成 … 見本の直線(straight_line) / ダイクストラ(go2_path_planning) / なし
  3) 経路追従(controller_server)を起動するか
  4) Gazebo GUI(3D窓)を出すか … 既定オフ(ヘッドレス)。重い時の固まり対策。可視化はRViz(#44)
  ※ plan_follower(橋渡し) と cmd_vel_safety(安全弁) は常時起動。
  ※ ゴール(/goal_pose)は自分で投げる(send_goal.sh)。
HELP
}

for arg in "$@"; do
    case "$arg" in
        --sim-nav2) SIM_NAV2=1 ;;
        --gate1) GATE1=1 ;;
        --no-build) DO_BUILD=0 ;;
        -h|--help) show_help; exit 0 ;;
        *) echo "不明なオプション: $arg"; echo; show_help; exit 1 ;;
    esac
done

if [ "$GATE1" = 1 ] && [ "$SIM_NAV2" = 1 ]; then
    echo "[NG] --gate1 と --sim-nav2 は同時指定できません(計測は本家Nav2を止める前提)。"
    exit 1
fi

# sim 本家Nav2は既定オフ(--sim-nav2でON)。
if [ "$SIM_NAV2" = 1 ]; then SIM_ENABLE_NAV2=true; else SIM_ENABLE_NAV2=false; fi
# 可視化は demo.launch.py の専用RViz(rviz/go2_demo.rviz)を使うため sim側RVizは常にオフ。
# 骨/経路(/plan)/AMCL結果/地図を1画面に出す設定。--gate1(計測)では可視化もオフ。
SIM_ENABLE_RVIZ=false
if [ "$GATE1" = 1 ]; then USE_RVIZ=false; else USE_RVIZ=true; fi

ask() {  # ask "質問" "既定(Y/N)"; 返り値: 0=はい 1=いいえ
    local q="$1" def="$2" ans
    read -rp "$q " ans
    ans="${ans:-$def}"
    [[ "$ans" =~ ^[YyＹ] ]]
}

# --- docker チェック -------------------------------------------------------
if ! docker info >/dev/null 2>&1; then
    echo "[NG] docker が使えません。ターミナルを開き直したか確認してください。"
    echo "     (即席で試すなら: newgrp docker してから再実行)"
    exit 1
fi

# --- 対話: 何を上げるか ----------------------------------------------------
echo "=== 何を起動するか選んでください =========================="
echo "[1] 自己位置推定 (/go2_localization/tf の供給元)"
echo "    1) 実装済み推定 (EKF/AMCL, go2_localization)  ← 既定"
echo "    2) 自作 (起動しない。自分で /go2_localization/tf を出す)"
read -rp "選択 [1]: " loc_ans
if [ "${loc_ans:-1}" = "2" ]; then USE_LOC=false; else USE_LOC=true; fi

echo "[2] 経路生成 (/plan の供給元)"
echo "    1) 見本: 直線 (straight_line_planner)  ← 既定"
echo "    2) ダイクストラ: NavFn曲線 (go2_path_planning。地図/自己位置推定が前提)"
echo "    3) 起動しない (自作を別途)"
read -rp "選択 [1]: " pl_ans
case "${pl_ans:-1}" in
    2) PLANNER=dijkstra ;;
    3) PLANNER=none ;;
    *) PLANNER=straight ;;
esac

if ask "[3] 経路追従(controller_server)を起動する? [Y/n]:" Y; then
    USE_FOLLOW=true; else USE_FOLLOW=false; fi

# Gazebo GUI(3D窓)は iGPU描画で重く、自律スタック+RViz と同時だと固まりやすい(#44)。
# 既定オフ(ヘッドレス)で可視化はRVizに寄せる。ロボットのメッシュや世界を見たい時だけ y。
if [ "$USE_RVIZ" = false ]; then
    SIM_GUI=false   # RViz無し(GATE1等)なら Gazebo GUI を出す意味は薄いが、既定オフで最軽量
elif ask "[4] Gazebo GUI(3D窓)も出す? 重い/固まる時はn推奨(可視化はRViz) [y/N]:" N; then
    SIM_GUI=true; else SIM_GUI=false; fi

echo "=========================================================="
echo "  自己位置推定=$([ "$USE_LOC" = true ] && echo 実装済み || echo 自作) / 経路生成=$PLANNER / 経路追従=$USE_FOLLOW"
if [ "$PLANNER" = dijkstra ] && [ "$USE_LOC" = false ]; then
    echo "  ※ ダイクストラは地図(/go2_localization/map)とTFが要ります。自作localizationがそれらを出す前提です。"
fi
echo "  sim本家Nav2=$SIM_ENABLE_NAV2 / Gazebo GUI=$SIM_GUI / RViz(専用go2_demo.rviz)=$USE_RVIZ$([ "$GATE1" = 1 ] && echo ' (GATE1計測モード)')"
echo ""

# --- sim / dev コンテナ起動 ------------------------------------------------
echo "=== 1/4 コンテナ起動 ==="
(cd docker/sim && SIM_ENABLE_NAV2="$SIM_ENABLE_NAV2" SIM_ENABLE_RVIZ="$SIM_ENABLE_RVIZ" SIM_GUI="$SIM_GUI" docker compose up -d) \
    || { echo "[NG] sim 起動に失敗(/dev/dri 関連なら docker/sim/compose.override.yaml.example 参照)"; exit 1; }
(cd docker && docker compose up -d) \
    || { echo "[NG] dev 起動に失敗(/dev/dri 関連なら docker/compose.override.yaml.example 参照)"; exit 1; }
echo "  [OK] $SIM_CONTAINER / $DEV_CONTAINER を起動"

# --- ロボット起動待ち ------------------------------------------------------
echo "=== 2/4 sim のロボット起動待ち(最大3分) ==="
DEADLINE=$((SECONDS + 180))
until docker exec "$SIM_CONTAINER" bash -c 'source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash && timeout 10 ros2 node list --no-daemon 2>/dev/null' | grep -q quadruped_controller; do
    if [ "$SECONDS" -ge "$DEADLINE" ]; then
        echo "[NG] 3分待ってもロボットが上がりません → docker logs $SIM_CONTAINER を確認"
        exit 1
    fi
    sleep 5
done
echo "  [OK] ロボット(歩容ノード)起動を確認"

# --- colcon build ----------------------------------------------------------
if [ "$DO_BUILD" = 1 ]; then
    echo "=== 3/4 colcon build (差分ビルド。--no-build で省略可) ==="
    docker exec "$DEV_CONTAINER" bash -c 'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --symlink-install' \
        || { echo "[NG] colcon build に失敗。上のログを確認してください"; exit 1; }
    echo "  [OK] ビルド完了"
else
    echo "=== 3/4 colcon build スキップ(--no-build) ==="
fi

# --- 起動後ガイド ----------------------------------------------------------
echo ""
echo "=== 4/4 自作スタックを前景起動します(Ctrl-C で停止) ======="
echo "▼ このあとログがこのターミナルに流れます。操作は別ターミナルで:"
echo ""
echo "  ● コンテナに入る:"
echo "      docker exec -it $DEV_CONTAINER zsh"
echo ""
echo "  ● ゴールを投げる(/goal_pose は自分で出してください):"
echo "      ~/ros2_ws/src/go2_path_following/scripts/send_goal.sh 3.0 0.0      # 3m 前方へ"
echo "      ~/ros2_ws/src/go2_path_following/scripts/send_goal.sh 2.0 1.5 90   # 斜め先で左向き"
if [ "$PLANNER" = none ]; then
echo ""
echo "  ● 経路生成は起動していません。自分のプランナを別ターミナルで起動してください。"
echo "    見本(直線)を出すだけなら(自作TFへの remap が必須):"
echo "      ros2 run straight_line_planner straight_line_planner_node \\"
echo "        --ros-args -p use_sim_time:=true -r /tf:=/go2_localization/tf"
fi
echo "=========================================================="
echo ""

LAUNCH_ARGS="use_localization:=$USE_LOC planner:=$PLANNER use_following:=$USE_FOLLOW use_rviz:=$USE_RVIZ"
exec docker exec -it "$DEV_CONTAINER" bash -c \
    "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch $DEMO_LAUNCH $LAUNCH_ARGS"
