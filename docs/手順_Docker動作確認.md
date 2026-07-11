# Docker環境 動作確認手順

`docker/` のROS2 Humble開発環境の動作確認手順。
Mac（§1〜§4）・Ubuntu（§5）とも動作確認済み。

- 環境の概要・パッケージ一覧は `docker/README.md` を参照
- ビルド済みイメージ: `arbeit-ros2:humble`（約5.5GB、Mac版ビルド・動作確認済み 2026-07-10／Ubuntu(x86_64)版ビルド・動作確認済み 2026-07-12）

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

## 5. Ubuntu機での利用（確認済み 2026-07-12）

Ubuntu 22.04 (x86_64, iGPU/AMD, NVIDIA無し) の開発PC実機で確認済み。
同じリポジトリを clone して `docker/` をそのまま使う。イメージはCPUアーキテクチャが違うので**再ビルドが必要**（Dockerfileは共通・変更不要）。

```bash
cd docker
docker compose build     # x86_64向けに再ビルド（10〜20分）
docker compose up -d
docker compose exec ros2 bash
```

Ubuntuで良くなること:

| 項目 | Mac | Ubuntu |
|------|-----|--------|
| RViz2 / Gazebo のGUI | XQuartz経由・低速 | ネイティブX11・GPU支援あり（下記設定） |
| 実機Go2とのDDS通信 | 制約あり（host networkingが不完全） | `network_mode: host` で素直に通る |
| Isaac Sim / Isaac Lab | 不可 | NVIDIA GPU搭載機なら可（コンテナとは別途セットアップ） |

Ubuntu側で必要な設定（`docker/compose.yaml` に反映済み・既定で有効）:

```bash
# GUI転送（ホストで一度実行。再起動すると再実行が必要）
xhost +local:docker
```

- `network_mode: host` は有効化済み（実機通信・GUIとも設定不要で通る）
- GUI用 volume `/tmp/.X11-unix:/tmp/.X11-unix` と `DISPLAY=${DISPLAY}` は有効化済み
  （Mac用の `host.docker.internal:0` は compose.yaml 内にコメントとして残置）
- iGPU（`/dev/dri`）はNVIDIA不要でdevicesとして渡し込み済み。実測でOpenGL 4.6が有効になり、
  RViz2・Gazebo(Ignition)ともソフトウェアレンダリングでなくGPU支援で動作することを確認した
- NVIDIA GPU搭載機で使う場合のみ `nvidia-container-toolkit` をホストに導入し、
  compose の `gpus: all` のコメントを外す（本機はNVIDIA非搭載のため未実施・未検証）

### 5.1 Ubuntu実機での確認結果（2026-07-12）

`docker compose build` でのイメージ再ビルドから、§2・§3の全項目を実施し以下を確認した。

| 確認項目 | 結果 |
|---------|------|
| `ros2 doctor` | All 5 checks passed |
| `ros2 pkg list \| wc -l` | 359（想定通り） |
| talker/listener pub/sub | 送受信OK |
| TF (`static_transform_publisher` / `tf2_echo`) | Translation [0, 0, 0.3] を正しく配信・受信 |
| `nav2_planner` / `nav2_controller` 実行ファイル | `planner_server` / `controller_server` とも確認 |
| `robot_localization` `ekf_node` | エラーなく起動・待機 |
| `teleop_twist_keyboard` | 実行ファイルの存在を確認（対話的なキー入力テストは非対話環境のため未実施。実機・GUI環境で別途確認のこと） |
| RViz2 (GUI) | `xhost +local:docker` 後に起動確認。ログ上で `OpenGl version: 4.6 (GLSL 4.6)` を確認し、iGPU支援でウィンドウ生成まで到達（ソフトウェアレンダリングにフォールバックしていない） |
| Gazebo (Ignition Fortress 6.18.0) | `ign gazebo -s -r shapes.sdf` でサーバ起動・物理エンジン(dartsim)ロードを確認。**注意: このHumble向けros_gzでは `gz` コマンドは存在せず `ign gazebo` を使う**（新しい `gz sim` 系コマンドとは名前が異なるので手順・スクリプトで注意） |

未実施（実機・別環境が必要なため）:

- 実機Go2とのDDS通信（`network_mode: host` 化はこの検証で完了。実機接続はC2で改めて実施）
- NVIDIA GPU機での `gpus: all` 検証（本機はNVIDIA非搭載のため対象外）
- go2_ros2_sim_py の実速度検証（`docker要件定義.md` 未決事項。sim/driverコンテナ自体が未着手）

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
- 2026-07-12: Ubuntu機（22.04, x86_64, iGPU）で§5を実施。compose.yamlをUbuntu既定設定に更新（network_mode: host・X11・/dev/dri）し、§2・§3全項目 + RViz2 + Gazebo(Ignition)の起動を確認
