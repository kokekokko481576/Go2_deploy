# Docker環境 動作確認手順

`docker/` のROS2 Humble開発環境の動作確認手順。
まずMacで一通り確認し（§1〜§4）、Ubuntu機が用意できたら§5に進む。

- 環境の概要・パッケージ一覧は `docker/README.md` を参照
- ビルド済みイメージ: `arbeit-ros2:humble`（約5.5GB、ビルド・動作確認済み 2026-07-10）

## 1. 起動と終了

```bash
# Docker Desktop が起動していなければ先に起動しておく（メニューバーにクジラアイコン）

cd ~/Arbeit/docker

# コンテナ起動（バックグラウンド）
docker compose up -d

# 中に入る（これを実行したターミナルがコンテナ内のシェルになる）
docker compose exec ros2 bash

# 終わるとき（コンテナの外・ホスト側で）
docker compose stop
```

- プロンプトが `ros@<コンテナID>:~$` になっていればコンテナ内
- `ROS_DISTRO` 等は自動で設定済み（`.bashrc` でsource済み）。手動のsourceは不要
- コンテナ内から抜けるだけなら `exit`（コンテナは動き続ける）

うまくいかないとき:

| 症状 | 対処 |
|------|------|
| `Cannot connect to the Docker daemon` | Docker Desktop が起動していない。起動して30秒ほど待つ |
| `no such service: ros2` | `docker/` ディレクトリ以外で compose を実行している。`cd ~/Arbeit/docker` してから |
| イメージがない・壊れた | `docker compose build` で再ビルド（10〜20分）。ネットワーク断で失敗したら再実行（リトライ設定済み・キャッシュで途中から再開） |

## 2. 基本動作確認（5分）

コンテナ内で:

```bash
# ROS2が使えるか
ros2 doctor            # → "All N checks passed" が出ればOK
                       #   (コンテナ内なのでネットワーク系のwarningが出ることはあるが問題ない)
ros2 pkg list | wc -l  # → 359前後
# 詳細レポートが見たいときは: ros2 doctor --report | less
# (head にパイプすると BrokenPipeError が出るが無害)
```

### pub/sub通信（ターミナル2つ使う）

ターミナルA（コンテナ内）:

```bash
ros2 run demo_nodes_cpp talker
# → [INFO] [talker]: Publishing: 'Hello World: 1' が1秒ごとに出れば送信OK
```

ターミナルB（ホストで新しいタブを開き、コンテナに入る）:

```bash
cd ~/Arbeit/docker && docker compose exec ros2 bash
ros2 run demo_nodes_cpp listener
# → [INFO] [listener]: I heard: [Hello World: N] が出れば受信OK
```

両方 `Ctrl+C` で止める。

## 3. 計画に関係する部分の予行演習（10分）

### 3.1 teleop → cmd_vel（経路追従M1の入口）

経路追従計画M1「テレオペでcmd_vel走行確認」の、実機なし版。

ターミナルA:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=前進 j/l=旋回 J/L=横移動(vy) k=停止
```

ターミナルB:

```bash
ros2 topic echo /cmd_vel
# キーを押すたびに Twist (linear.x, linear.y, angular.z) が流れるのを確認
# → 実機ではこの /cmd_vel をドライバがGo2のMove命令に変換する
```

### 3.2 主要ツールの起動確認

```bash
# EKF（自己位置推定M1で使う）— 設定なしでも起動だけ確認
ros2 run robot_localization ekf_node --ros-args -p use_sim_time:=false
# → エラーなく待機状態になればOK（Ctrl+Cで終了）

# Nav2のライフサイクル確認（経路生成M2/経路追従M2で使う）
ros2 pkg executables nav2_planner
ros2 pkg executables nav2_controller
# → planner_server / controller_server が表示されればOK
```

### 3.3 TFツール

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link lidar_link &
ros2 run tf2_ros tf2_echo base_link lidar_link
# → Translation: [0.000, 0.000, 0.300] が出ればTF配信OK（自己位置推定3.1のTF構成の最小形）
kill %1
```

## 4. GUI（RViz2）を使いたい場合（Mac・任意）

Macでは XQuartz 経由になる（重い。本格的なGUI作業はUbuntu推奨）。

1. XQuartz をインストール: `brew install --cask xquartz` → 一度ログアウト/再起動
2. XQuartz を起動し、環境設定 → セキュリティ → 「ネットワーク・クライアントからの接続を許可」にチェック → XQuartz再起動
3. ホストのターミナルで: `xhost +localhost`
4. コンテナ内で: `rviz2`

ウィンドウが出れば成功。描画が崩れる・遅いのは仕様（GPU支援なし）。

## 5. Ubuntu機での利用（明日以降）

同じリポジトリを clone して `docker/` をそのまま使う。イメージはCPUアーキテクチャが違うので**再ビルドが必要**（Dockerfileは共通・変更不要）。

```bash
git clone <このリポジトリ> && cd Arbeit/docker
docker compose build     # x86_64向けに再ビルド
docker compose up -d
docker compose exec ros2 bash
```

Ubuntuで良くなること:

| 項目 | Mac | Ubuntu |
|------|-----|--------|
| RViz2 / Gazebo のGUI | XQuartz経由・低速 | ネイティブX11・GPU支援あり（下記設定） |
| 実機Go2とのDDS通信 | 制約あり（host networkingが不完全） | `network_mode: host` で素直に通る |
| Isaac Sim / Isaac Lab | 不可 | NVIDIA GPU搭載機なら可（コンテナとは別途セットアップ） |

Ubuntu側で必要な設定:

```bash
# GUI転送（ホストで一度実行）
xhost +local:docker
```

`docker/compose.yaml` の変更点:

- `network_mode: host` のコメントを外す（実機通信・GUI とも楽になる）
- GUI用に volume `- /tmp/.X11-unix:/tmp/.X11-unix` と `DISPLAY=$DISPLAY` を有効化
  （Mac用の `host.docker.internal:0` はUbuntuでは使わない）
- NVIDIA GPUをコンテナから使う場合は `nvidia-container-toolkit` をホストに導入し、
  compose に `gpus: all` を追加（Gazeboの物理・描画が速くなる）

※ compose.yaml にUbuntu用設定のコメントを併記してあるので、コメントの付け替えだけで切り替わる。

## 6. よく使うコマンド早見表

| やりたいこと | コマンド（ホスト側 `docker/` で） |
|-------------|----------------------------------|
| 起動 | `docker compose up -d` |
| 入る | `docker compose exec ros2 bash` |
| 停止 | `docker compose stop` |
| コンテナ破棄（コードは消えない） | `docker compose down` |
| 再ビルド | `docker compose build` |
| ログ確認 | `docker compose logs` |

| やりたいこと | コマンド（コンテナ内で） |
|-------------|------------------------|
| ワークスペースのビルド | `cd ~/ros2_ws && colcon build --symlink-install` |
| ビルド結果の反映 | `source ~/ros2_ws/install/setup.bash`（新しいシェルなら自動） |
| トピック一覧 | `ros2 topic list` |
| ノード一覧 | `ros2 node list` |
| TFツリー確認 | `ros2 run tf2_tools view_frames`（frames.pdf が生成される） |

## 履歴

- 2026-07-10: 初版作成（Mac動作確認済み。Ubuntu節は未実施・明日以降に検証）
