#!/usr/bin/env bash
# Gazebo真値publisher(Issue #43)をsimコンテナへ送り込んで起動する。
# 使い方: ./docker/sim/tools/start_ground_truth.sh
#
# 【重要】これはsim限定のデバッグ/計測ツール。/gz_ground_truth/pose(自己位置推定とは
# 別の名前空間)に真値をpublishするだけで、自己位置推定システムには一切干渉しない。
# 実機移行時はdocker/sim/tools/を削除するだけで取り除ける。
set -eu
SIM_CONTAINER=go2-sim
THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker exec "$SIM_CONTAINER" pkill -f /tmp/ground_truth_publisher.py 2>/dev/null || true
docker cp "$THIS_DIR/ground_truth_publisher.py" "$SIM_CONTAINER:/tmp/ground_truth_publisher.py"
docker exec -d "$SIM_CONTAINER" bash -c \
  "source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash && \
   python3 /tmp/ground_truth_publisher.py \
   > /tmp/ground_truth_publisher.log 2>&1"
echo "[OK] gz_ground_truth_publisher を起動しました(/gz_ground_truth/pose)"
echo "     ログ: docker exec $SIM_CONTAINER cat /tmp/ground_truth_publisher.log"
