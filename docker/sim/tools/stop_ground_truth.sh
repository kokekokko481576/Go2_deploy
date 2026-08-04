#!/usr/bin/env bash
# ground_truth_publisher.py(Issue #43)を止める。
set -eu
SIM_CONTAINER=go2-sim
docker exec "$SIM_CONTAINER" pkill -f /tmp/ground_truth_publisher.py 2>/dev/null \
  && echo "[OK] 停止しました" \
  || echo "[INFO] 起動していませんでした"
