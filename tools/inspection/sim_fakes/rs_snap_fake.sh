#!/bin/bash
# 撮影の代役（sim の通し試験専用）。inspect_run.py が確かめる logs/rs/<名前>_color.png を作るだけ。
# 中身は画像ではない（1行のテキスト）。試験の後に消すこと（sim_inspect_full.sh が消す）。
set -eu
name=${1:?名前}
out=${RS_OUT:-~/marker_detection/logs/rs}
mkdir -p "$out"
echo "sim fake snapshot $(date)" > "$out/${name}_color.png"
echo "保存: $out/${name}_color.png (fake)"
