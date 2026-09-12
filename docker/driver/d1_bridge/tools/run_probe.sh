#!/bin/bash
# dds_probe を安全に起動するラッパー。
#
# **LD_LIBRARY_PATH で /usr/local/lib を先頭に置くこと。** ROS2 Humble の
# CycloneDDS(/opt/ros/humble/lib)が先に見つかると、unitree_sdk2 純正の
# CycloneDDS とABIが食い違い "free(): invalid pointer" で落ちる
# (2026-08-28 に実機で踏んだのと同じ症状。d1_sdk/run.sh と同じ対策)。
#
# 使い方: /root/d1_bridge_tools/run_probe.sh [購読する秒数(既定30)]
set -e
BIN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/build/dds_probe"
if [ ! -x "$BIN" ]; then
    echo "エラー: $BIN がありません。イメージを再ビルドしてください" >&2
    exit 1
fi
# CYCLONEDDS_URI は setup_dds.sh が書いた /tmp/cyclonedds.xml を指す。
# 実機なし(GO2_NIC未指定)なら lo + ユニキャスト探索になっており、
# 同じコンテナ内のROS2ノードと相互に発見できる。
exec env LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}" "$BIN" "$@"
