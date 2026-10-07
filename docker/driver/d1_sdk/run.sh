#!/bin/bash
# D1アーム サンプル実行ラッパー
#
# unitree_sdk2純正のCycloneDDS(/usr/local/lib)より、ROS2 Humbleの
# CycloneDDS(/opt/ros/humble/lib/...)がLD_LIBRARY_PATHで優先されると
# ABI不整合で "free(): invalid pointer" クラッシュを起こすため、
# /usr/local/lib を優先させてから実行する。
#
# 使い方:
#   ./run.sh <実行ファイル名> [引数...]
#   例: ./run.sh get_arm_joint_angle
#   例: ./run.sh joint_angle_control
#
# 引数なしで実行するとビルド済みの実行ファイル一覧を表示する。

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

if [ ! -d "$BUILD_DIR" ]; then
    echo "エラー: ビルドディレクトリが見つかりません: $BUILD_DIR" >&2
    echo "先に以下でビルドしてください:" >&2
    echo "  cd $SCRIPT_DIR && mkdir -p build && cd build && cmake .. && make" >&2
    exit 1
fi

if [ $# -eq 0 ]; then
    echo "使い方: $0 <実行ファイル名> [引数...]"
    echo ""
    echo "ビルド済みの実行ファイル:"
    find "$BUILD_DIR" -maxdepth 1 -type f -executable -printf '  %f\n' | sort
    exit 0
fi

TARGET="$1"
shift

BIN="$BUILD_DIR/$TARGET"
if [ ! -x "$BIN" ]; then
    echo "エラー: 実行ファイルが見つかりません: $BIN" >&2
    echo "" >&2
    echo "ビルド済みの実行ファイル:" >&2
    find "$BUILD_DIR" -maxdepth 1 -type f -executable -printf '  %f\n' | sort >&2
    exit 1
fi

exec env LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}" "$BIN" "$@"
