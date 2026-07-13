# ROS2 Humble 開発環境（Go2 経路生成・経路追従・自己位置推定 用）
# 対象: 作業計画 Phase 0 C1（ROS2環境構築）
# Apple Silicon (arm64) / x86_64 の両方でビルド可能
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ダウンロードの一時的な接続断に耐えるようリトライを設定
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries

# --- OSRF リポジトリ追加 ---
# Gazebo (Fortress / ros_gz) の arm64 版は Ubuntu 標準リポジトリだと古く
# 依存解決に失敗する(libignition-sensors6 6.8.0 < 6.8.1)ため、OSRF公式を追加
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# --- 開発ツール + ROS2パッケージ ---
# 各計画書との対応:
#   navigation2 / nav2-bringup      : 経路生成 M2〜(planner server)・経路追従 M2〜(controller server)
#   robot-localization              : 自己位置推定 M1(脚オドメトリ+IMUのEKF融合)
#   slam-toolbox                    : 自己位置推定 C4(事前スキャン地図の作成)
#   pointcloud-to-laserscan         : 自己位置推定 M2(顎LiDAR点群→2D LaserScan変換→AMCL)
#   grid-map                        : 経路生成 M4(2.5D標高マップ)
#   ros-gz                          : 検証計画(Gazeboシミュレーション先行)。Humble対応の
#                                     新Gazebo(Fortress)。Classic(gazebo11)はarm64版が無い
#   teleop / joy                    : 経路追従 M1(テレオペでのcmd_vel走行確認)
#   rmw-cyclonedds-cpp              : unitree_ros2 が CycloneDDS 前提
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    vim \
    less \
    gdb \
    iputils-ping \
    net-tools \
    tree \
    htop \
    tmux \
    wget \
    unzip \
    jq \
    ripgrep \
    fd-find \
    bat \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-grid-map \
    ros-humble-ros-gz \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    ros-humble-tf2-tools \
    ros-humble-rqt-graph \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu の fd-find/bat パッケージは実行ファイル名が fdfind/batcat になるため、
# 通常名(fd/bat)でも呼べるようにリンクしておく
RUN ln -s $(command -v fdfind) /usr/local/bin/fd \
    && ln -s $(command -v batcat) /usr/local/bin/bat

# --- 非rootユーザ ---
ARG USERNAME=ros
ARG UID=1000
RUN useradd -m -u ${UID} -s /bin/bash ${USERNAME} \
    && apt-get update && apt-get install -y --no-install-recommends sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# rosdep 初期化（ユーザ側 update のみ。init はベースイメージ済み）
RUN rosdep update

# シェル起動時に ROS2 とワークスペースを自動 source
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc

CMD ["bash"]
