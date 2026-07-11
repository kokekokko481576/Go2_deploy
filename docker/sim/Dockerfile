# simコンテナ（Gazebo + Go2モデル + Nav2）
# 対象: docs/docker要件定義.md §4.1「sim: go2_ros2_sim_py の流用方針」
#
# 本体(external/go2_ros2_sim_py)は変更せず追従可能に保つ。
# このDockerfileはこちら側の差分(ビルドレシピ)のみを持つ。
#
# 注意: upstream純正の docker/Dockerfile は colcon-cache が
# ワークスペースの .git HEAD を解決できることを前提にしている。
# 本プロジェクトでは go2_ros2_sim_py を git submodule として取り込んでおり、
# submodule内の .git はコンテナに複製すると無効な参照になり colcon-cache が
# 「Ref 'HEAD' did not resolve to an object」で失敗する。そのため
# colcon-cache は使わず、素の colcon build のみを行う。
#
# ROS2ディストロ: go2_ros2_sim_py は Jazzy 前提(README記載)。プロジェクト全体の
# Humble固定(unitree_ros2の都合)とは別に、simコンテナはJazzy+Gazebo Harmonicで
# 独立して動かす。Nav2互換の標準メッセージ(geometry_msgs/Twist等)でdev/driver側と
# やり取りする分にはディストロが違っても問題ない。
ARG BASE_IMG=ros:jazzy
FROM ${BASE_IMG}

ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-urdf \
    ros-jazzy-xacro \
    ros-jazzy-std-msgs \
    ros-jazzy-angles \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-controller-manager \
    ros-jazzy-hardware-interface \
    ros-jazzy-pluginlib \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2launch \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-camera-info-manager \
    ros-jazzy-rqt-robot-steering \
    ros-jazzy-tf-transformations \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-opennav-docking \
    ros-jazzy-apriltag-ros \
    ros-jazzy-apriltag-msgs \
    ros-jazzy-image-proc \
    ros-jazzy-image-view \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2 \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# go2_ros2_sim_py 本体(submodule)をワークスペースへ。colcon-cacheを使わないため
# submodule内の .git (gitlink、コンテナ内では無効な参照になる)が含まれても実害はない
COPY external/go2_ros2_sim_py /root/ws/src

WORKDIR /root/ws

SHELL ["/bin/bash", "-c"]

RUN . /ros_entrypoint.sh && \
    colcon build --symlink-install

ENV WORKSPACE_DIR=/root/ws
RUN sed -i '/exec "\$@"/i umask 002' /ros_entrypoint.sh \
    && sed -i '/exec "\$@"/i source "$WORKSPACE_DIR/install/setup.bash"' /ros_entrypoint.sh

CMD ["bash"]
