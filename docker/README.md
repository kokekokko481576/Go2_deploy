# ROS2 Humble 開発用 Docker 環境

作業計画 Phase 0「C1 ROS2環境構築」用のコンテナ。
ROS2 Humble + Nav2 + robot_localization + slam_toolbox + Gazebo など、
3計画（自己位置推定・経路生成・経路追従）で使うパッケージを一式含む。

## できること / できないこと

| 項目 | 可否 |
|------|------|
| ROS2 Humble での開発・ビルド（colcon） | ○ |
| Nav2（planner / controller server）の構成・検証 | ○ |
| Gazebo シミュレーション（新 Gazebo = Fortress。コマンドは `ign gazebo`、`gz` ではない） | ○ Ubuntu+iGPU / △ Mac（GPU なし・低速） |
| RViz2 などの GUI | ○ Ubuntu（ネイティブX11・iGPUでOpenGL 4.6を確認） / △ Mac（XQuartz 経由） |
| Isaac Sim / Isaac Lab | × NVIDIA GPU 必須。本イメージ・本開発機（iGPUのみ）では不可（GPU 搭載 Linux 機で別途） |
| 実機 Go2 との DDS 通信 | ○ Ubuntu（`network_mode: host` 有効化済み） / △ Mac（ホストネットワーク設定に制約） |

## 使い方

```bash
# 初回: イメージのビルド（数GBダウンロード、10〜20分）
cd docker
docker compose build

# 起動して中に入る
docker compose up -d
docker compose exec ros2 bash

# 動作確認（コンテナ内）
ros2 run demo_nodes_cpp talker    # 別タブで listener
```

ワークスペースはリポジトリ直下の `ros2_ws/` がコンテナ内 `~/ros2_ws` にマウントされる。
コンテナを破棄してもコードはホスト側に残る。

```bash
# コンテナ内でのビルド
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## GUI（RViz2 / Gazebo）を使う場合

### Ubuntu（確認済み・既定設定）

1. ホストのターミナルで `xhost +local:docker`（再起動後は再実行が必要）
2. コンテナ内で `rviz2` または `ign gazebo`（Fortressのコマンドは `gz` ではなく `ign gazebo`）

`DISPLAY=${DISPLAY}` と `/tmp/.X11-unix` マウント、iGPU(`/dev/dri`)渡し込みは compose.yaml で有効化済み。
iGPU機での実機確認で `OpenGl version: 4.6` を確認済み（ソフトウェアレンダリングにフォールバックしない）。

### macOS

1. XQuartz をインストールして起動（`brew install --cask xquartz`）
2. XQuartz の設定 →「セキュリティ」→「ネットワーク・クライアントからの接続を許可」を有効化して再起動
3. ホストのターミナルで `xhost +localhost`
4. compose.yaml の DISPLAY 行を Mac 用（`host.docker.internal:0`、コメントで残置）に切り替えてから `rviz2`

## 実機 Go2 と通信する場合

- unitree_ros2 は CycloneDDS 前提（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 設定済み）。
  unitree_ros2 自体は `ros2_ws/src` に clone して colcon build する（同梱していない）
- DDS のマルチキャストがコンテナ境界を越えられないため、`network_mode: host` が必要
  - Ubuntu: compose.yaml で有効化済み（確認済み）
  - macOS: Docker Desktop 4.34+ の "Enable host networking" を有効にする必要があり、制約が残る。
    実機接続は Linux 機（ネイティブ or Docker --net=host）を推奨

## 含まれる主なパッケージと計画書の対応

| パッケージ | 用途（計画書の該当箇所） |
|-----------|------------------------|
| navigation2 / nav2-bringup | 経路生成 M2（planner server）、経路追従 M2（controller server: MPPI/DWB） |
| robot_localization | 自己位置推定 M1（脚オドメトリ + IMU の EKF 融合） |
| slam_toolbox | 共通基盤 C4（事前スキャン地図の作成） |
| pointcloud_to_laserscan | 自己位置推定 M2（顎LiDAR点群 → 2D LaserScan → AMCL） |
| grid_map | 経路生成 M4（2.5D 標高マップ） |
| ros_gz（Gazebo Fortress） | 検証（Gazebo シミュレーション先行の方針）。Classic(gazebo11) は arm64 版が存在しないため新 Gazebo を採用 |
| teleop_twist_keyboard / joy | 経路追従 M1（テレオペでの cmd_vel 走行確認） |
| rmw_cyclonedds_cpp | unitree_ros2 の DDS 要件 |
