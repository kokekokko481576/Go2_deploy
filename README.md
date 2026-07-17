# Go2_deploy

Unitree Go2（顎3D LiDAR・背面アーム搭載）を用いて、船内部材（防撓材等）へ移動し
溶接・検査作業を行うロボットシステムを構築するプロジェクト。
自己位置推定・経路生成・経路追従の3つのサブシステムを統合し、ROS2 + Nav2 ベースで実装する。

## 全体像

```mermaid
flowchart LR
    A[自己位置推定<br>EKF + AMCL] --> D[Nav2 統合]
    B[経路生成<br>直線プランナ→A*→高さコスト] --> D
    C[経路追従<br>cmd_vel安全機構→MPPI/DWB] --> D
    D --> E[部材への正対・最終アプローチ]
    E --> F[アーム作業<br>溶接・検査]
```

開発は Isaac Lab（学習）→ Gazebo（リハーサル）→ 実機Go2（unitree_ros2経由）の順で
シミュレーション先行を原則とする。詳細は `docs/go2_rl_ros2_overview.md`（超入門ガイド）を参照。

## ディレクトリ構成

| パス | 内容 |
|------|------|
| `docs/` | 計画ドキュメント一式（索引は`docs/README.md`。3計画・統合作業計画・開発ガイド等） |
| `docker/` | ROS2 Humble 開発環境（dev）、実機ドライバ環境（`docker/driver/`）、Gazeboシミュレーション環境（`docker/sim/`） |
| `ros2_ws/` | colcon ワークスペース。`build/`・`install/`・`log/` は gitignore 済み |
| `external/` | 外部リポジトリの git submodule（`unitree_ros2`、`go2_ros2_sim_py`） |
| `chapter1/` | AI-Robot-Book 教材のサブモジュール（`update=none, ignore=all`。触らない） |

## セットアップ

```bash
git clone --recurse-submodules <このリポジトリのURL>
```

### 開発環境（dev: ROS2 Humble + Nav2）

```bash
cd docker
docker compose build   # 初回のみ
docker compose up -d
docker compose exec ros2 zsh
```

詳細（GUI表示・実機通信の設定等）は [`docker/README.md`](docker/README.md) を参照。

### 実機ドライバ環境（driver: unitree_ros2）

```bash
cd docker/driver
docker compose build
docker compose up -d
docker compose exec driver zsh
```

詳細は [`docker/driver/README.md`](docker/driver/README.md) を参照。

### シミュレーション環境（sim: Gazebo + Go2モデル + Nav2）

```bash
cd docker/sim
docker compose build sim
xhost +local:docker
docker compose up -d
```

詳細は [`docker/sim/README.md`](docker/sim/README.md) を参照。

### ワークスペースのビルド

```bash
# devコンテナ内(zshが既定。ROS2は自動source済み)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.zsh
```

## 実装済みパッケージ（`ros2_ws/src/`）

| パッケージ | 内容 |
|-----------|------|
| [`straight_line_planner`](ros2_ws/src/straight_line_planner) | 経路生成 Phase1 M1。自己位置から目標作業姿勢までの直線補間Pathを出す最小プランナ。Gazebo到達確認済み |
| [`cmd_vel_safety`](ros2_ws/src/cmd_vel_safety) | 経路追従 M1。速度・加速度クランプとウォッチドッグ（0.5s）による安全機構 |
| [`go2_localization`](ros2_ws/src/go2_localization) | 自己位置推定 M1/M2。robot_localization(EKF)+nav2_amcl の設定・起動一式 |
| [`go2_path_following`](ros2_ws/src/go2_path_following) | 経路追従 M2。Nav2 controller_server(DWB)の最小構成+Path→FollowPathの橋渡しノード |
| [`fake_localization_sensors`](ros2_ws/src/fake_localization_sensors) | Gazebo・実機なしでEKF確認するためのダミーOdometry/IMU Publisher |

## ドキュメント

`docs/`配下のドキュメント一覧・読む順番は [`docs/README.md`](docs/README.md) を参照。

現在の進捗は `docs/作業計画.md` の「履歴」節を正とする。実際に手を動かす手順は
`docs/開発ガイド.md` を参照。

## コミット規約

コミットメッセージは和文セマンティック形式（`<type>: <絵文字> <#issue番号> <和文の要約>`）で書く。
詳細は [`CLAUDE.md`](CLAUDE.md) を参照。
