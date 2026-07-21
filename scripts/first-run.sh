#!/usr/bin/env bash
# Go2_deploy: 初回ビルド+起動+疎通チェック (Issue #28/#29/#30)
# リポジトリ直下で: ./scripts/first-run.sh
# やること: sim/devの両イメージをビルド → 起動 → DDS疎通の自動チェック。
# 2回目以降の実行も安全(ビルドはキャッシュが効く)。

set -u
cd "$(dirname "$0")/.."
FAIL=0
ok() { echo "  [OK] $1"; }
ng() { echo "  [NG] $1"; FAIL=1; }

if ! docker info >/dev/null 2>&1; then
    echo "[NG] dockerが使えません。wsl2-setup.sh の後にターミナルを開き直しましたか?"
    echo "     (即席で試すなら: newgrp docker してからこのスクリプトを再実行)"
    exit 1
fi

echo "=== 1/3 ビルドと起動(初回は合計15〜40分。コーヒーでも) ==="
(cd docker/sim && docker compose build sim && docker compose up -d) || { ng "simのビルド/起動に失敗(/dev/dri関連なら docker/sim/compose.override.yaml.example を参照)"; exit 1; }
(cd docker && docker compose build && docker compose up -d) || { ng "devのビルド/起動に失敗(/dev/dri関連なら docker/compose.override.yaml.example を参照)"; exit 1; }
ok "go2-sim / arbeit-ros2 を起動した"

echo "=== 2/3 Gazebo起動待ち(最大3分) ==="
DEADLINE=$((SECONDS + 180))
until docker exec go2-sim bash -c 'source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash && timeout 10 ros2 node list --no-daemon 2>/dev/null' | grep -q quadruped_controller; do
    if [ "$SECONDS" -ge "$DEADLINE" ]; then
        ng "3分待ってもロボットが上がらない → docker logs go2-sim を確認して #28 に報告"
        break
    fi
    sleep 5
done
[ "$FAIL" -eq 0 ] && ok "sim内でロボット(歩容ノード)起動を確認"

echo "=== 3/3 dev⇔simのDDS疎通チェック(#30) ==="
if docker exec arbeit-ros2 bash -c 'source /opt/ros/humble/setup.bash && timeout 20 ros2 topic list --no-daemon 2>/dev/null' | grep -q '/robot1/'; then
    ok "devコンテナから /robot1/* トピックが見える"
else
    ng "devコンテナからsimのトピックが見えない → #30 に詳細を報告"
fi
if docker exec arbeit-ros2 bash -c 'source /opt/ros/humble/setup.bash && timeout 15 ros2 topic echo /robot1/odometry/filtered --once --no-daemon 2>/dev/null' | grep -q 'frame_id'; then
    ok "実データ(odometry)がdev側に届いている"
else
    ng "トピックは見えるがデータが届かない → #30 に詳細を報告"
fi

echo ""
if [ "$FAIL" -eq 0 ]; then
    echo "=== 全チェック通過 ==="
    echo "残る確認は目視1つ: GazeboのGUIウィンドウ(カフェ+ロボット)が出ていれば #29 もクリア。"
    echo "次の遊び方: ros2_ws/src/go2_path_following/README.md の「使い方(フェーズB)」"
else
    echo "=== NGあり。メッセージ内のissueに詰まりどころを記録してください ==="
fi
exit "$FAIL"
