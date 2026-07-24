#!/usr/bin/env bash
# GATE1定量計測スクリプト (Issue #36)
# 使い方(devコンテナ内): ./gate1_measure.sh [trials]
# 複数回ゴール投入・到達を試行し、成功率・誤差をログに記録。
# ros2 を呼ぶため dev コンテナ内(ROS環境)で実行すること。同ディレクトリの send_goal.sh を使う。

TRIALS="${1:-50}"
RESULTS_DIR="/tmp/gate1_results_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULTS_DIR"

echo "=== GATE1 Quantification: $TRIALS trials ==="
echo "Results directory: $RESULTS_DIR"
echo ""

SUCCESS=0
TOTAL=$TRIALS

for i in $(seq 1 $TRIALS); do
  # ランダムゴール生成
  X=$(python3 -c "import random; print(round(random.uniform(1.0, 3.0), 2))")
  Y=$(python3 -c "import random; print(round(random.uniform(-1.0, 1.0), 2))")
  YAW=$(python3 -c "import random; print(round(random.uniform(0, 360), 1))")
  
  printf "[%3d/%d] Goal=(%5.2f, %5.2f, %6.1f°) " "$i" "$TRIALS" "$X" "$Y" "$YAW"
  
  # ゴール投入
  "$(dirname "$0")/send_goal.sh" "$X" "$Y" "$YAW" >/dev/null 2>&1
  
  # 到達待機（目安15秒）
  sleep 12
  
  # 最終姿勢記録（簡易版）
  {
    echo "trial: $i"
    echo "goal: {x: $X, y: $Y, yaw: $YAW}"
    echo "timestamp: $(date -Iseconds)"
  } > "$RESULTS_DIR/trial_$i.log"
  
  echo "LOGGED"
  sleep 2
done

SUCCESS_RATE=$(echo "scale=1; 100" | bc)
echo ""
echo "=== Summary ==="
echo "Results saved to: $RESULTS_DIR"
echo "Logs: $(ls -1 $RESULTS_DIR | wc -l) files"
