#!/bin/bash
# rs_snap.cpp を Go2 背中の Jetson へ送ってビルドし直す（2026-09-28）。
#   ~/marker_detection/tools/rs_snap_install.sh
# 今の実行ファイルは ~/riku_rs/rs_snap.bak_<日時> に残す（戻すときは mv で戻す）。
# Jetson はネットに出られないので、ROS noetic 同梱の librealsense2 (2.50) にリンクする。
set -euo pipefail
J=unitree@192.168.123.18
HERE=$(cd "$(dirname "$0")" && pwd)
scp -q "$HERE/rs_snap.cpp" "$J:riku_rs/rs_snap.cpp"
ssh "$J" 'set -e; cd ~/riku_rs
  [ -f rs_snap ] && cp rs_snap rs_snap.bak_$(date +%m%d_%H%M%S)
  g++ -O2 -std=c++14 rs_snap.cpp -o rs_snap.new -I/opt/ros/noetic/include \
      -L/opt/ros/noetic/lib/aarch64-linux-gnu -lrealsense2 \
      -Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu
  mv rs_snap.new rs_snap
  echo "ビルドしました: ~/riku_rs/rs_snap"; ls -la rs_snap*'
echo "確認: ~/marker_detection/tools/rs_snap.sh --depth 848x480 test_848"
