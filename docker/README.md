# ROS2 Humble 開発用 Docker 環境

作業計画 Phase 0「C1 ROS2環境構築」用のコンテナ。
ROS2 Humble + Nav2 + robot_localization + slam_toolbox + Gazebo など、
3計画（自己位置推定・経路生成・経路追従）で使うパッケージを一式含む。

## できること / できないこと

Windows(WSL2) 列は #28 のスクリプト整備までは完了し、**実 Windows ハードウェアでの
検証を進行中**の暫定値（🔍=検証中）。確定したら本表を更新する。

| 項目 | Ubuntu | macOS | Windows (WSL2) |
|------|--------|-------|----------------|
| ROS2 Humble での開発・ビルド（colcon） | ○ | ○ | 🔍 WSL2内Ubuntu 22.04+Docker Engine（Linuxと等価の想定） |
| Nav2（planner / controller server）の構成・検証 | ○ | ○ | 🔍 同上（sim⇔dev構成で検証中） |
| Gazebo シミュレーション（新 Gazebo = Fortress。コマンドは `ign gazebo`、`gz` ではない） | ○ iGPU | △ GPU なし・低速 | 🔍 WSLg経由。GPUドライバ次第で D3D12 描画/ソフトレンダリング（#29） |
| RViz2 などの GUI | ○ ネイティブX11・iGPUでOpenGL 4.6を確認 | △ XQuartz 経由 | 🔍 WSLg（`/tmp/.X11-unix` パススルー、xhost不要の想定）（#29） |
| Isaac Sim / Isaac Lab | × iGPUのみで不可 | × | × NVIDIA GPU必須。WSL2 GPU対応可否は調査中（#33） |
| 実機 Go2 との DDS 通信 | ○ `network_mode: host` 有効化済み | △ ホストネットワーク設定に制約 | 🔍 `networkingMode=mirrored`＋Hyper-Vファイアウォール設定が別途必要な見込み（#31） |

> **Windows の前提**: WSL2ネイティブ運用（WSL2内にDocker Engineを入れ、WSL2のLinuxシェルから
> `docker compose` を実行。Docker Desktopは使わない）。リポジトリは必ず WSL2側のLinux
> ファイルシステム（`~/` 以下）にcloneすること（`/mnt/c/...` はCRLF混入・bind mount低速化のため不可）。
> セットアップは `docs/手順/Windows-WSL2セットアップ.md`（スクリプト3本を上から実行するだけ）。
> sim⇔dev の DDS 疎通（#30）・GUI 表示（#29）は自己診断スクリプトで確認でき、結果を各 issue に記録する。

## 使い方

```bash
# 初回: イメージのビルド（数GBダウンロード、10〜20分）
cd docker
docker compose build

# 起動して中に入る
docker compose up -d
docker compose exec ros2 zsh

# 動作確認（コンテナ内）
ros2 run demo_nodes_cpp talker    # 別タブで listener
```

ワークスペースはリポジトリ直下の `ros2_ws/` がコンテナ内 `~/ros2_ws` にマウントされる。
コンテナを破棄してもコードはホスト側に残る。

```bash
# コンテナ内でのビルド(cb は後述のエイリアス)
cd ~/ros2_ws
cb
source install/setup.zsh
```

## 対話シェル(zsh)

対話シェルの既定は `zsh`。予測入力・補完メニュー・シンタックスハイライト・starshipプロンプトを
`docker/common/zshrc`(dev/sim/driver共通、ホストの `~/.zshrc` を参考に軽量に再現)で設定済み。

- `ll`/`la`/`l`: エイリアス(ホストの `aliases.zsh` と同じ)
- `cb` / `cbp <pkg>`: `colcon build --symlink-install` / `--packages-select <pkg>`
- ROS2は`ROS_DISTRO`・ワークスペースとも起動時に自動source済み(手動sourceは通常不要)
- 従来の`bash`も引き続き使える(`docker compose exec ros2 bash`)。`.bashrc`のROS2 sourceはそのまま残置

詳細・他コンテナ(sim/driver)との共通化については `docker/common/` 参照。

## GUI（RViz2 / Gazebo）を使う場合

### Ubuntu（確認済み・既定設定）

1. ホストのターミナルで `xhost +local:docker`（再起動後は再実行が必要）
2. コンテナ内で `rviz2` または `ign gazebo`（Fortressのコマンドは `gz` ではなく `ign gazebo`）

`DISPLAY=${DISPLAY}` と `/tmp/.X11-unix` マウント、iGPU(`/dev/dri`)渡し込みは compose.yaml で有効化済み。
iGPU機での実機確認で `OpenGl version: 4.6` を確認済み（ソフトウェアレンダリングにフォールバックしない）。

### Windows (WSL2)

WSLg 経由で自動的に X11 が通る（`DISPLAY` と `/tmp/.X11-unix` は WSL2 が用意）。
compose.yaml の Ubuntu 設定がそのまま流用でき、`xhost` は通常不要。
詳細・トラブルシュートは `docs/手順/Windows-WSL2セットアップ.md` を参照（検証中・#29）。

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
