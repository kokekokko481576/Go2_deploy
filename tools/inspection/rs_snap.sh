#!/bin/bash
# Go2 背中の D435i で1枚撮り、この PC へ持ってきて PNG にし、画面に出す（2026-09-24）。
#   ~/marker_detection/tools/rs_snap.sh [--depth 848x480] [名前]   名前を省くと日時（例 snap_0924_1215）
# 保存先: ~/marker_detection/logs/rs/<名前>_both.png ほか
#
# --depth: 深度の撮影解像度（2026-09-28 追加）。省略時は 1280x720（最短測距 約0.28m）。
#   848x480 にすると最短測距が縮み、約0.2mのアップ撮影でも深度が取れる見込み（実機で要確認）。
#   保存される深度はカラーに位置合わせされるので、どちらでも 1280x720 で出る。
#   環境変数 RS_DEPTH=848x480 でも指定できる（inspect_run.py --depth はこれを使う）。
set -euo pipefail
depth=${RS_DEPTH:-}
if [ "${1:-}" = "--depth" ]; then
  depth=$2; shift 2
fi
name=${1:-snap_$(date +%m%d_%H%M%S)}
out=~/marker_detection/logs/rs
mkdir -p "$out"
dargs=""
if [ -n "$depth" ]; then
  dargs="${depth%x*} ${depth#*x}"
fi
ssh unitree@192.168.123.18 "~/riku_rs/rs_snap ~/riku_rs/$name $dargs"
scp -q "unitree@192.168.123.18:riku_rs/${name}_*" "$out/"
png=$(python3 "$(dirname "$0")/rs_view.py" "$out/$name")
echo "保存: $png"
xdg-open "$png" >/dev/null 2>&1 &
