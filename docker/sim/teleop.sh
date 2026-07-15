#!/bin/bash
# go2-simコンテナ内でteleop_twist_keyboardを起動し、/robot1/cmd_velへリマップする。
# docker execは/ros_entrypoint.shを経由しないためROS環境が未sourceの状態になる。
# ここで明示的にsourceしてからros2を呼ぶ。
set -e
exec docker exec -it go2-sim bash -c \
    '. /opt/ros/jazzy/setup.bash && . "$WORKSPACE_DIR/install/setup.bash" && \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel'
