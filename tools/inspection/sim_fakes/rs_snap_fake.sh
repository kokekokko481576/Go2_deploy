#!/bin/bash
# 撮影の代役（sim の通し試験専用）。inspect_run.py が確かめる logs/rs/<名前>_color.png を作る。
# SIM_RENDER=1 のときは、アームの代役の角度から部材の深度画像を合成する（render_snap.py。
# aim_correct.py の閉ループ試験用）。それ以外は中身の無い1行のテキスト。
# 試験の後に消すこと（sim_inspect_full.sh が消す）。
set -eu
name=${1:?名前}
out=${RS_OUT:-~/marker_detection/logs/rs}
mkdir -p "$out"
if [ "${SIM_RENDER:-0}" = 1 ]; then
  exec python3 "$(dirname "$0")/render_snap.py" "$name"
fi
echo "sim fake snapshot $(date)" > "$out/${name}_color.png"
echo "保存: $out/${name}_color.png (fake)"
