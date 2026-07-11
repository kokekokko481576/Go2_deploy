# Docker環境 要件定義

## 1. 目的

Go2ナビゲーション開発(自己位置推定・経路生成・経路追従の3計画)の実行環境をDockerで統一する。ホストOSに依存せずROS2 Humbleを固定し、「開発PCで作ったものがJetsonでそのまま動く」状態を作る。本リポジトリはそのDocker構成の管理用。

## 2. 前提

- ROS2ディストロ: **Humble固定**。理由: unitree_ros2(実機ドライバ)の対応がHumbleまで。Jazzy/Lyricalに実機ドライバが出たら再評価(2026-07時点で存在しない)
- ホスト: 開発PC = Linux(GPU非力、iGPU想定)/ 本番 = Go2 EduのJetson(JetPack、arm64)
- 通信: CycloneDDS(Go2ファームはCycloneDDS 0.10.x固定。バージョン整合はドライバイメージ側で吸収する)
- シミュレータ: Gazebo(当面)。Isaac Sim/Labは別担当・本リポジトリのスコープ外

## 3. 要件

- R1 開発PC(amd64)とJetson(arm64)の両方でビルド・実行できること
- R2 全コンテナ `--net=host` で起動し、DDSがホストネットワークに直接参加できること。`ROS_DOMAIN_ID` と `CYCLONEDDS_URI`(NIC指定)は環境変数で注入
- R3 RViz等のGUIがX11透過で表示できること(`DISPLAY` + `/tmp/.X11-unix` マウント)
- R4 GPU: NVIDIA必須にしないこと。iGPUは `/dev/dri` の渡し込みで対応。NVIDIA環境ではnvidia-container-toolkitをオプションで有効化できる二段構え
- R5 自作ノードのワークスペースはbind mountし、colcon buildはコンテナ内で行う(イメージ再ビルドなしで開発ループが回ること)
- R6 実機接続時のズッコケ防止: cmd_velウォッチドッグ等の安全ノード(経路追従M1)はドライバコンテナに同居させる

## 4. 構成方針: 外部流用と自作の切り分け

| コンテナ | 中身 | 方針 |
|---------|------|------|
| sim | Gazebo + Go2モデル + Nav2 | **外部流用**: [go2_ros2_sim_py](https://github.com/abutalipovvv/go2_ros2_sim_py) |
| driver | unitree_ros2 + CycloneDDS 0.10.x | **自作**(公式 .devcontainer を参考に) |
| dev | 自作ノード群 + Nav2 + slam_toolbox | **自作**(薄い) |

### 4.1 sim: go2_ros2_sim_py の流用方針

本体はいじらず、こちら側のcompose overrideで差分を持つ:

- upstream はforkせずサブモジュールまたはDockerfile内cloneで取り込み、追従可能に保つ
- NVIDIA前提になっている箇所(nvidia runtime指定)をoverrideで無効化し、`/dev/dri` を渡すiGPU構成を既定にする。NVIDIA機用のoverrideも別ファイルで用意
- 歩容が自前Python IK(実機と別物)である点は許容。cmd_vel I/FだけがNav2検証に必要な契約
- 弱GPU向け設定(ヘッドレス起動、2D LaserScan代用、センサレート削減)はこちらのlaunch/設定で持つ

### 4.2 driver: 自作イメージ

`ros:humble` ベースに unitree_ros2 と CycloneDDS 0.10.x をビルドして焼き込む。要点:

- CycloneDDSのバージョンをGo2ファームに合わせてピン留め(ここがJazzy移行を阻む本丸なので、イメージに閉じ込めて誰も触らなくて良くする)
- amd64 / arm64 のマルチアーチビルド(開発PCでのテレオペと、Jetsonデプロイの両用)
- NIC名(`enp*` / `eth*`)は環境変数で注入し、イメージには焼かない

### 4.3 dev: 自作ノード用イメージ

`ros:humble` + apt で navigation2, slam_toolbox, robot_localization 等を追加した薄いイメージ。自作ソースはイメージに含めず bind mount(R5)。

## 5. 段階導入

1. sim コンテナ単体で Gazebo + Nav2 が回る(経路生成/追従のシミュ検証を開始できる)
2. dev コンテナから sim に対して自作ノード(直線プランナ等)が動く
3. driver コンテナを開発PCで実機接続テスト(テレオペ = 経路追従M1)
4. driver + dev をJetsonでビルド・常駐化(本番形)

## 6. 未決事項

- JetsonのJetPackバージョン未確認(arm64ベースイメージの選定に影響。l4tベースが必要になる可能性)
- ~~go2_ros2_sim_py がiGPUで実用速度か未検証~~ → **解消(2026-07-12)**。§7参照。
  実用速度を確認できたため次点(go2_nav_simulation)への切替は不要と判断
- Isaac Sim側との統合テスト環境をどちらのリポジトリが持つか(担当者と要調整)

## 7. 進捗（段階導入 §5 対応）

- §5-1「sim コンテナ単体で Gazebo + Nav2」: **達成(2026-07-12)**。
  `external/go2_ros2_sim_py` を submodule として取り込み、`docker/sim/` に独自のビルドレシピ
  (upstream純正Dockerfileはcolcon-cache起因のsubmodule非互換のため使わず、素のcolcon buildで構成。
  詳細は `docker/sim/README.md`)を用意し、Gazebo(Harmonic)+Go2モデル+Nav2フルスタックの起動を確認。
  リアルタイム係数は平均概ね1.0前後(0.5〜1.4で推移)、コンテナのメモリ使用量は約1.4GB/14.9GB(9%)で、
  本開発PCのiGPU(AMD Radeon 760M)でも実用速度と判断
  - simコンテナはROS2 **Jazzy**固定(go2_ros2_sim_py自体の前提)。dev/driverのHumbleとは別ディストロで
    独立して動く構成とした(標準メッセージ型はディストロを跨いで相互運用可能なため契約上は問題なし)
- §5-2「devコンテナからsimに対して自作ノードが動く」: **未着手**(自作ノード自体が未実装のため)
- §5-3「driverコンテナを開発PCで実機接続テスト」: **イメージビルド・ループバックでの動作確認まで完了(2026-07-12)**。
  `external/unitree_ros2` を submodule として取り込み、`docker/driver/` に自作Dockerfile
  (要件定義どおり公式.devcontainerを参考にした軽量版。詳細は `docker/driver/README.md`)を用意し、
  `unitree_go`/`unitree_api`/`unitree_hg`/`unitree_ros2_example` のビルドと `ros2 run` での
  サンプルノード起動、ループバック経由のpub/sub発見を確認した。**実機Go2への有線LAN接続は未実施**(次段階)
  - upstream(`unitree_ros2_example`)のCMakeLists.txtにインストール先指定漏れがあり、
    実行ファイルが`ros2 run`から見つけられない不備を発見。本体は変更せずDockerfile側の
    後処理(シンボリックリンク)で対処
  - ループバックでの動作確認向けに、CycloneDDSのユニキャスト発見設定(`AllowMulticast=false`+
    `Peers`+`ParticipantIndex=auto`)が必要と判明(`lo`はmulticast非対応のため)。実機NIC使用時は
    通常のmulticast発見が使える想定でこの分岐には入らない
- Phase 0 C1(ROS2環境構築)は Ubuntu 実機で動作確認済み(2026-07-12、詳細は `手順_Docker動作確認.md` §5)。
  `network_mode: host`・X11・iGPU(`/dev/dri`)渡し込みをUbuntu既定として compose.yaml に反映済み
- Ignition Gazebo(Fortress、Humble向けros_gz)のコマンドは `gz` ではなく `ign gazebo` である点に注意。
  一方 go2_ros2_sim_py側(Jazzy、Gazebo Harmonic)は `gz` コマンド系である(バージョン・パッケージ系統が別)
