# driverコンテナ（unitree_ros2 + CycloneDDS 0.10.x固定）
# 対象: docs/docker要件定義.md §4.2「driver: 自作イメージ」
#
# 本体(external/unitree_ros2)は変更せず追従可能に保つ。
# 公式 .devcontainer/Dockerfile-humble はzsh/oh-my-zsh/clang-18等の対話開発向けツールを
# 大量に含み、かつコンテナ内でビルドを完結させない(sleepループで待機し手動ビルドが前提)ため、
# ここでは要件定義どおり「公式.devcontainerを参考に」した軽量な自作Dockerfileとする
# (ビルドをコンテナ内で完結させる点は変えていない)。
# なお対話シェルの使い勝手向上のためzshは追加で導入している(下記・docker/common/参照)が、
# oh-my-zsh/clang-18等の重量級ツール一式は引き続き入れていない。
#
# CycloneDDSバージョン: Go2ファーム側は0.10.2固定(unitree_ros2 README記載)。
# README曰く「Humbleの場合はCycloneDDSのソースビルドは不要、apt版のros-humble-rmw-cyclonedds-cppで良い」
# とのことなので、Foxy向けのソースビルド手順(README手順2)は行わない。
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rosbag2 \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    iproute2 \
    tree \
    htop \
    tmux \
    curl \
    wget \
    unzip \
    jq \
    ripgrep \
    fd-find \
    bat \
    zsh \
    zsh-autosuggestions \
    zsh-syntax-highlighting \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu の fd-find/bat パッケージは実行ファイル名が fdfind/batcat になるため、
# 通常名(fd/bat)でも呼べるようにリンクしておく
RUN ln -s $(command -v fdfind) /usr/local/bin/fd \
    && ln -s $(command -v batcat) /usr/local/bin/bat

# プロンプト(starship)。単一の静的バイナリで依存が無く、ビルドを重くしない
RUN curl -sS https://starship.rs/install.sh | sh -s -- --yes

# unitree_ros2本体(submodule)のうち、実際にビルドが必要な部分だけ取り込む:
#   cyclonedds_ws/src/unitree : unitree_go/unitree_api/unitree_hg メッセージ定義
#   example/src               : サンプルノード群(unitree_go/api/hgのみに依存。unitree_sdk2は不要)
COPY external/unitree_ros2/cyclonedds_ws/src/unitree /root/ws/src/unitree
COPY external/unitree_ros2/example/src /root/ws/src/unitree_ros2_example

WORKDIR /root/ws
SHELL ["/bin/bash", "-c"]

RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# unitree_ros2_example の CMakeLists.txt (upstream, submodule) は
# install(TARGETS ... DESTINATION) の指定が不備でパッケージルート直下に
# 実行ファイルをインストールしてしまい、`ros2 run`/`ros2 pkg executables` から
# 見つけられない(バイナリ自体は正常)。本体は変更せず、ここでlib/以下に
# シンボリックリンクを張って ros2 run で使えるようにする。
RUN mkdir -p /root/ws/install/unitree_ros2_example/lib/unitree_ros2_example && \
    find /root/ws/install/unitree_ros2_example -maxdepth 1 -type l -executable | \
    xargs -I{} ln -s {} /root/ws/install/unitree_ros2_example/lib/unitree_ros2_example/

COPY docker/driver/setup_dds.sh /setup_dds.sh
COPY docker/driver/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# docker compose exec 等の対話シェル(entrypointを経由しない)でもROS2・CycloneDDS設定が
# 効くよう、.bashrc/.zshrc両方からも同じ設定をsourceする(docker/Dockerfileの既存方針と同様)。
RUN echo 'source /setup_dds.sh' >> ~/.bashrc

# --- 対話シェルはzshを既定に(docker/common/参照。予測入力・補完メニュー・starshipプロンプト) ---
COPY docker/common/zshrc /root/.zshrc
COPY docker/common/starship.toml /root/.config/starship.toml
RUN echo 'source /setup_dds.sh' >> ~/.zshrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["zsh"]
