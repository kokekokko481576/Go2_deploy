# driverコンテナ（unitree_ros2 + CycloneDDS 0.10.x固定）

対象: `docs/docker要件定義.md` §4.2「driver: 自作イメージ」、作業計画 Phase 0 C2「ドライバ導入」。
外部リポジトリ [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) を
`external/unitree_ros2` に git submodule として取り込み、本体は変更せず利用する。

## できること

- `unitree_go` / `unitree_api` / `unitree_hg` メッセージ定義(Go2ファームとのDDS通信I/F)
- unitree_ros2公式のサンプルノード群(`unitree_ros2_example`。go2_sport_client等)
- CycloneDDS 0.10.x(apt版 `ros-humble-rmw-cyclonedds-cpp`)固定。NIC名は`GO2_NIC`環境変数で注入(イメージには焼かない、要件R2)
- `go2_sport_bridge`(自作): `cmd_vel`(経路追従の安全フィルタ出力)を工場出荷状態の歩容
  (Sport Mode API)のMove命令に変換して配信。Isaac Lab学習を使わず、devコンテナのテレオペ/Nav2の
  `cmd_vel`だけで実機を歩かせたい場合の橋渡し(下記「cmd_vel→工場出荷歩容ブリッジ」参照)

## cmd_vel→工場出荷歩容ブリッジ(`go2_sport_bridge`)

Isaac Labでの学習済み歩容を使わず、Go2工場出荷状態の歩容(Sport Mode API)を
ROS2の`cmd_vel`から直接動かすための最小構成。経路: `teleop_twist_keyboard`(dev)
→`cmd_vel_raw`→`cmd_vel_safety`(dev、クランプ+ウォッチドッグ)→`cmd_vel`
→(DDS、host network越し)→`go2_sport_bridge`(driver)→`/api/sport/request`(Move, api_id=1008)→実機。

devとdriverはどちらも`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`・`ROS_DOMAIN_ID=0`・
`network_mode: host`で揃えてあるため、`cmd_vel`はコンテナを跨いでそのまま届く
(`cmd_vel_safety`の実績と同じ仕組み。ただしdev/driverは両方Humbleなので
dev⇔sim(Jazzy)間で出ていたCycloneDDSのXTypes警告は出ない想定)。

```bash
# 1. driverコンテナ: 実機/ループバックに接続し、まず起立させる(工場出荷のStandUp)
cd docker/driver && docker compose up -d
docker compose exec driver ros2 run unitree_ros2_example go2_sport_client 4   # StandUp

# 2. driverコンテナ: cmd_vel→Move変換ブリッジを起動
docker compose exec driver ros2 run go2_sport_bridge cmd_vel_to_sport_node

# 3. devコンテナ: 安全フィルタとテレオペを起動
cd ../.. && cd docker && docker compose up -d
docker compose exec ros2 ros2 run cmd_vel_safety cmd_vel_safety_node
docker compose exec ros2 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
```

`go2_sport_bridge`は`cmd_vel`を20Hz(既定)で`/api/sport/request`のMoveへ変換して配信し続け、
0.5秒(既定)cmd_velが途絶えるとMove(0,0,0)にフォールバックするウォッチドッグを持つ
(`cmd_vel_safety`のウォッチドッグと二重の安全策。ロジックは
`docker/driver/bridge/go2_sport_bridge/cmd_vel_to_sport_node.py`参照)。

ループバック(`GO2_NIC`未指定)での動作確認は済み(下記「動作確認結果」)。
StandUp前にMoveを送っても効果が無い/意図しない挙動の可能性があるため、
起立(`go2_sport_client 4`または`1`=BalanceStand)を先に必ず実行すること。

### 非常停止と単軸ジョグ

```bash
# 1軸だけ短時間動かして符号・速さを確かめる(初回は必ずこれから)
docker compose exec driver ros2 run go2_sport_bridge jog.sh vx 0.20 1.0

# 非常停止。別ターミナルで常に打てる状態にしてから走らせること
docker compose exec driver ros2 run go2_sport_bridge estop.sh
```

`jog.sh`は`/cmd_vel`を直接叩く(dev側の`cmd_vel_safety`を経由しない)。
`estop.sh`はブリッジを`pkill`してから停止指令を3秒間直接送る。**ブリッジを落とすだけでは
止まらない**(機体が最後の指令のまま歩き続ける恐れがある)ため、この順序が要る。

### 実機で踏んだ罠(2026-09-02 実測、別プロジェクトでの検証結果を反映)

ループバック検証では出ない、**実機でしか現れない**問題。ブリッジ実装(`cmd_vel_to_sport_node.py`)
は以下を織り込んである。

- **リクエストIDを毎回変えないと機体が動かない**。同じ`header.identity.id`で同じ内容を送り
  続けると、機体が重複指令とみなして無視する。指令値が完全に一定になった瞬間から実速度が
  0.0000m/sになり、値が変わった瞬間だけ動く、という形で現れる。テレオペでキーを押しっぱなしに
  すると`Twist`が一定値になるため、まさにこの条件に入る。unitree_ros2同梱の例(`cmd_vel_control.cpp`)
  は`id`を設定しないので、そのままでは連続的な速度制御ができない
- **機体が「通常モード」でないとMoveを受けても歩かない**。起立させただけでは足りない。
  DDS上は指令が正常に流れているように見えるため、原因の切り分けが難しい
- **StopMoveを連投してはいけない**。20Hzで送り続けると機体の移動指令の受け付けを妨げ、
  リモコン操作とも競合する。ウォッチドッグ作動時は「遷移時にStopMoveを1回」+
  「以降はゼロ速度のMoveのみ」にしてある
- **0.15m/s程度がGo2の歩容の下限**。それ未満は胴体が揺れるだけで前に進まない。
  「まず低速から」と思って0.1m/s以下で試すと「動かない」と誤認する
- **横移動(`linear.y`)はほとんど効かない。それどころか前進を殺す**。`vy=-0.056`程度でも
  実速度がゼロになった。正対に近い接近では`vy`を0にして前進と旋回だけで組むこと
- **`obstacles_avoid`経由(障害物回避あり)は物理リモコンを奪う**。回避を効かせるには
  `UseRemoteCommandFromApi(true)`の送信が要るが、これを送ると物理リモコンが操作不能になった
  (電源再投入または`UseRemoteCommandFromApi(false)`で復帰)。リモコンは最後の非常停止手段
  なので、本ブリッジは`/api/sport/request`経路のみを使い、機体側の設定には一切触らない

## 使い方

```bash
cd docker/driver
docker compose build   # 初回のみ
docker compose up -d
docker compose exec driver zsh   # 対話シェル(既定はzsh)。RMW/CycloneDDS設定は.zshrc経由で自動適用

# 実機Go2に有線LAN接続する場合(ホストで一度)
GO2_NIC=enp3s0 docker compose up -d
```

## 本体(upstream)との差分・注意点

- **`unitree_ros2_example` の `CMakeLists.txt`(upstream)に不備**: `install(TARGETS ...)` の
  destination指定が欠けており、実行ファイルがパッケージルート直下にインストールされ、
  `ros2 run`/`ros2 pkg executables` から見つけられない(バイナリ自体は正常)。
  本体は変更せず、Dockerfile側でビルド後に `lib/unitree_ros2_example/` へシンボリックリンクを
  張る後処理を追加して対処(`docker/driver/Dockerfile` 参照)
- **entrypoint.sh とCycloneDDS/RMW設定を共通化**: `/setup_dds.sh` に集約し、コンテナのメイン
  プロセス(entrypoint経由)と `docker compose exec` の対話シェル(`.bashrc`/`.zshrc` 経由)の
  いずれからも同じ設定が当たるようにしている。`setup_dds.sh` 内のROS2 setup読み込みは
  `$ZSH_VERSION` の有無でbash/zsh版を切り替えている(setup.bashは`BASH_SOURCE`等bash固有の
  構文を含みzshからsourceすると失敗するため)。`docker exec` で非対話(`bash -c`、`-i`無し)に
  入ると先頭ガード(`[ -z "$PS1" ] && return` 相当)により`.bashrc`/`.zshrc`の残りが読まれず
  設定が当たらない点に注意(対話シェルなら問題ない)
- **ループバック(`GO2_NIC`未指定=既定`lo`)でのDDS発見**: `lo` はmulticastフラグが
  立っておらず(本機で確認済み)、CycloneDDS既定のマルチキャストSPDP発見が機能しない。
  `setup_dds.sh` 内で `lo` の場合のみ `AllowMulticast=false` + `Peers`(ユニキャスト)+
  `ParticipantIndex=auto` に切り替えて対処(`ParticipantIndex=none`のままだとPeer側から
  ポートを特定できず発見できない)。**実機NIC使用時はこの分岐に入らず素の既定設定**
  (multicastが使える前提)になる

## 動作確認結果(2026-07-12、開発PC: Ubuntu22.04)

- イメージビルド成功(`unitree_go`/`unitree_api`/`unitree_hg`/`unitree_ros2_example` 4パッケージ)
- `ros2 doctor`: All 5 checks passed
- `ros2 pkg executables unitree_ros2_example`: 19実行ファイルすべて認識(修正後)
- `ros2 run unitree_ros2_example go2_stand_example`: 起動しセンサ読み取りループが回ることを確認(実機無しなのでゼロ値)
- ループバック(`GO2_NIC`未指定)でのpub/sub: `ros2 topic pub`→`ros2 topic echo`/`ros2 topic list`で
  受信・トピック発見を確認(daemon再起動後。上記のDDS発見設定込み)

未実施(実機・別環境が必要なため):

- 実機Go2との有線LAN接続でのDDS通信(`GO2_NIC`に実NIC名を指定しての検証)
- `unitree_ros2_example` の各サンプル(sport_client等)を実機相手に実行しての動作確認

## `go2_sport_bridge` 動作確認結果(2026-08-03、開発PC: Ubuntu22.04、ループバック)

- イメージビルド成功(`go2_sport_bridge`パッケージ追加。`ros2 pkg executables go2_sport_bridge`で
  `cmd_vel_to_sport_node`を確認)
- `ros2 topic pub /cmd_vel ... -r 20`で`linear.x=0.3, angular.z=0.2`を注入 →
  `/api/sport/request`に`api_id=1008`・`parameter={"x": 0.3, "y": 0.0, "z": 0.2}`が
  配信されることを`ros2 topic echo`で確認
- publishを止めてから0.5秒後、ウォッチドッグが作動し`parameter={"x": 0.0, "y": 0.0, "z": 0.0}`へ
  自動で切り替わることを確認

未実施(実機が必要なため):

- 実機Go2への有線LAN接続でのMove配信・起立→歩行の実挙動確認(Issue #3)
- devコンテナの`cmd_vel_safety`・`teleop_twist_keyboard`からのコンテナ跨ぎ結合(dev⇔driver。
  dev⇔sim(Jazzy)での実績はあるが、dev⇔driver間では今回は未実施)

## `go2_sport_bridge` 動作確認結果(2026-09-04、開発PC: Ubuntu22.04、ループバック、#71)

実機実測を反映した改修(リクエストID・StopMoveの扱い・終了時停止・速度クランプ)の検証。

- `ros2 pkg executables go2_sport_bridge`に`estop.sh`・`jog.sh`が実行ファイルとして
  登録されること(`ros2 run`から呼べること)を確認
- **リクエストIDが毎回変わること**を確認。`linear.x=0.3, angular.z=0.2`を固定値で
  20Hz注入し続けても、`header.identity.id`が
  `1788495624411076906` → `...625361218165` → `...626261152899` と毎回異なる
  (`parameter`は同一)。**改修前は常に`id: 0`だった**
- 速度クランプを確認。`linear.x=0.9`を注入 → `parameter={"x": 0.3, ...}`
  (既定`max_vx=0.3`でクランプ)
- **StopMoveを連投しないこと**を確認。約14秒の観測でMove(1008)が502件に対し
  StopMove(1003)は3件のみ(ウォッチドッグ遷移2回 + 終了時1回)
- **終了時に停止指令が出ること**を確認。SIGINT送出後、最後に配信されたメッセージが
  StopMove(1003)であること、プロセスが残らず終了することを確認
- SIGTERM(`docker stop`相当)でも同様に終了することを確認
- `estop.sh`のE2E確認。ブリッジ稼働中に実行 → ブリッジのプロセスが消え、
  StopMove(1003)とゼロ速度Moveが配信されることを確認
- `estop.sh`が**購読者不在でもハングせず3.4秒で完走する**ことを確認
  (`ros2 topic pub --once`は既定で購読者を待つため`-w 0`が必須)

未実施(実機が必要なため): 上記はいずれもループバックでの確認であり、
**「機体が実際に歩くか」は実機でしか確認できない**(Issue #3)。
