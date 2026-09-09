# 実機Go2 接続チェックリスト(Issue #3 準備用)

対象: Issue #3「driverコンテナ: 実機Go2との有線LAN接続検証」。
実機・LANケーブルが揃うまでの間に**この開発PCだけで確認・準備できることを済ませておく**ためのメモ。
接続当日にやること(手順)と、その前にやれること(準備・検証済み事項)を分けて書く。

---

## 接続前に準備・検証済みのこと(実機なしで完了)

### 1. cmd_vel → 工場出荷歩容(Sport Mode API)ブリッジ

`docker/driver/bridge/go2_sport_bridge`(詳細は`docker/driver/README.md`)。
`cmd_vel`を購読し`/api/sport/request`のMove命令(api_id=1008)に変換する。
ループバックで単体動作を確認済み(値の変換・ウォッチドッグとも正常)。

### 2. dev⇔driverコンテナ間のDDS到達性(2026-08-03検証)

**問題**: dev(`docker/compose.yaml`)・driver(`docker/driver/compose.yaml`)は共に
`network_mode: host`だが、driverは`GO2_NIC`でNICを明示指定するのに対し、
devは何も指定しておらず、CycloneDDSの既定インタフェース自動選択に任せていた。
この開発PCのように**NICが複数あるマシン**(本機: `enp2s0`有線 + `wlp3s0`無線)では、
dev/driver各コンテナが別々のNICを自動選択し、**互いのトピックを発見できない**ことを
実際に確認した(`ros2 topic pub`をdriver側で実行してもdev側の`ros2 topic list`に出ない)。

**対処**: dev側(`docker/compose.yaml`)にもdriverと同様の`GO2_NIC`環境変数を追加し、
指定時はCycloneDDSのインタフェースを明示的に固定できるようにした
(`GO2_NIC`未指定時は今までどおりCycloneDDS既定動作のまま。sim連携等の既存動作は変えない)。
実機接続当日は**dev・driver両方に同じ`GO2_NIC=<実機と繋がるNIC名>`を指定して起動する**こと。

**検証方法と結果**: `enp2s0`はこの検証時点でIPv4アドレス未割当(後述)だったため、
代わりにIPを持つ`wlp3s0`を両コンテナに指定して同じ経路を再現。
`GO2_NIC=wlp3s0`でdev・driver双方を起動 →
dev側で`ros2 topic list`にdriver側のトピック(`/cmd_vel`, `/api/sport/request`)が
表示されることを確認 → dev側から`ros2 topic pub /cmd_vel ...`(vx=0.4, wz=-0.1)を実行 →
driver側の`go2_sport_bridge`が同じ値で`/api/sport/request`にMoveを配信することを確認。
**ブリッジ・DDS疎通の仕組み自体は実機なしで一通り検証済み**、実機当日に残る不確定要素は
「実機と繋がる物理NIC自体の疎通」のみに絞り込めた。

### 3. 発見した注意点: NICにIPv4アドレスが必要

`enp2s0`(リンクUPだがIPv4未割当)を指定すると、CycloneDDSが起動時に
`enp2s0: does not match an available interface` エラーで失敗することを確認した。
CycloneDDSはIPv4アドレスが割り当たっていないインタフェースを「利用可能」とみなさない。
→ unitree_ros2公式READMEが「NICを手動で固定IP(192.168.123.99/255.255.255.0)に設定する」
よう案内しているのはこのため。**接続当日、ケーブル接続後に固定IPを設定するまでは
CycloneDDSがそのNICで起動できない**点を踏まえて手順を組む必要がある。

---

## 接続当日の手順

### 実測結果(2026-09-04、初接続)

**DDS疎通までは通った。歩行(下記3)はまだ未実施。**

- ホストNIC: `enp2s0`、`192.168.123.51/24`(公式READMEの`.99`でなくても通る)
- Go2実機: `192.168.123.161`。`ping` 0% loss / 0.27ms
- driverコンテナから実機トピック **121本** を確認
  (`/sportmodestate` `/lowstate` `/utlidar/cloud` `/api/sport/request` `/wirelesscontroller` 他)
- 実データも届いている: バッテリ soc 66% / `power_v` 30.11V、`body_height` 0.321(起立中)
- `go2_sport_bridge` の `cmd_vel_to_sport_node` は起動成功
  (20Hz、watchdog 0.5s、上限 vx=0.3 vy=0.2 wz=0.5)。cmd_vel未受信時のウォッチドッグ作動も確認

### 0. 事前確認

- [ ] Go2本体の電源・起立可能な状態(地面に安全に置ける場所)を確保
- [ ] LANケーブルでGo2とこの開発PCを接続
- [ ] `ip -brief link` でリンクアップしたNIC名を確認(本機では`enp2s0`の見込み。
      ドックやアダプタ経由の場合は名前が変わりうるので都度確認)
- [ ] (LiDARキャリブレーション用)メジャーまたはノギス、スマホの水平器/傾斜計アプリを用意
      (下記「4. 顎3D LiDARの搭載位置キャリブレーション」で使用)
- [ ] (同上)平らな床と、正対できる平らな壁がある場所を確保できるか確認しておく

### 1. ホストNICに固定IPを設定

unitree_ros2公式README準拠: 該当NICのIPv4を手動設定でアドレス`192.168.123.99`、
サブネットマスク`255.255.255.0`に設定する。

```bash
# 例: nmcli で恒久的な接続プロファイルを作る場合(NetworkManager)
nmcli con add type ethernet ifname enp2s0 con-name go2-wired \
  ipv4.method manual ipv4.addresses 192.168.123.99/24
nmcli con up go2-wired

# もしくはその場限りの一時設定(再起動・NIC抜き差しで消える)
sudo ip addr add 192.168.123.99/24 dev enp2s0
```

- [ ] 設定後、`ip -4 addr show enp2s0` でアドレスが付いていることを確認
- [ ] `ping <Go2のIP>` で疎通確認(Go2側のIPは実機ラベル・アプリ・付属マニュアル等で要確認。
      unitree公式READMEには具体的な既定IPの記載が無いため、当日実機側で確認すること)

### 2. コンテナ起動(両方に同じNIC名を指定)

```bash
# driverコンテナ
cd docker/driver
GO2_NIC=enp2s0 docker compose up -d

# devコンテナ(別ターミナル)
cd docker
GO2_NIC=enp2s0 docker compose up -d
```

- [ ] driverコンテナ内で `ros2 topic list` に実機由来のトピック
      (`/sportmodestate`, `/lowstate` 等、unitree_ros2 README参照)が出ることを確認
- [ ] `ros2 doctor`: エラー無し

**`GO2_NIC` を渡し忘れると既定の `lo` で起動し、実機は一切見えない**(2026-09-04に踏んだ)。
`lo` のときは `setup_dds.sh` がユニキャスト探索(`AllowMulticast=false` + `Peer 127.0.0.1`)に
切り替わるため、実機のマルチキャストSPDPが届かない。`docker inspect go2-driver` の
`GO2_NIC` を見れば起動時の値が分かる。

**さらに紛らわしいのが `ros2 topic list` で、これは ros2 daemon のキャッシュを返す。**
`GO2_NIC=lo` で実機が見えていない状態でも、以前の探索結果を引きずって
実機トピック121本を表示した。**疎通確認は必ず `--no-daemon` を付けて行うこと**
(`lo` のままなら `/parameter_events` と `/rosout` の2本しか出ない)。
`ros2 topic hz` は `--no-daemon` を受け付けないので、確認は `topic list` / `topic echo` で行う。

```bash
docker exec go2-driver bash -c 'source /setup_dds.sh; ros2 topic list --no-daemon'
```

NICを直すときは `docker compose down` してから `up -d` する
(`--force-recreate` では古いコンテナが残ることがある)。

### 3. 起立 → cmd_velブリッジ起動 → テレオペ

**先に「起立させただけでは歩かない」ことを頭に入れておく。** 機体が「通常モード」でないと
Move命令を受け付けても歩かない。DDS上は指令が正常に流れて見えるため、これを知らないと
原因の切り分けで時間を溶かす(2026-09-02 実測)。

```bash
# driverコンテナ: 起立
docker compose exec driver ros2 run unitree_ros2_example go2_sport_client 4   # StandUp

# driverコンテナ: cmd_vel→Moveブリッジ
docker compose exec driver ros2 run go2_sport_bridge cmd_vel_to_sport_node

# driverコンテナ: **非常停止用**。別ターミナルで打てる状態にしてから先に進む
docker compose exec driver ros2 run go2_sport_bridge estop.sh
```

まず**テレオペではなく`jog.sh`で1軸ずつ**確認する。テレオペはキー1つで複数軸が同時に動き、
符号が逆だったときに何が起きたのか分からなくなる。

```bash
# driverコンテナ: 1軸ずつ(前進 / 左 / 左旋回)。正の向きは REP-103
docker compose exec driver ros2 run go2_sport_bridge jog.sh vx 0.20 1.0
docker compose exec driver ros2 run go2_sport_bridge jog.sh wz 0.30 1.0
```

- [ ] `jog.sh vx 0.20 1.0` で前進する(**0.15m/s未満では進まない**。下記「安全上の注意」参照)
- [ ] `jog.sh wz 0.30 1.0` で左(反時計回り)に旋回する
- [ ] 指令を止めてからウォッチドッグが作動し、Go2が停止することを確認
- [ ] `estop.sh` で確実に停止することを確認

1軸ずつの符号・速さが正しいことを確認してから、テレオペに進む。

```bash
# devコンテナ: 安全フィルタ
docker compose exec ros2 ros2 run cmd_vel_safety cmd_vel_safety_node \
  --ros-args -p max_linear_x:=0.22 -p max_linear_y:=0.18 -p max_angular_z:=0.45

# devコンテナ: テレオペ(別ターミナル)
docker compose exec ros2 ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=cmd_vel_raw
```

- [ ] キー入力でGo2が実際に前進・旋回することを確認
- [ ] テレオペを止めてから0.5秒程度でGo2が停止する(ウォッチドッグ)ことを確認

### 安全上の注意

- 周囲に十分なスペースを確保し、転倒・衝突しても問題ない環境で行う
- **`estop.sh`を別ターミナルで打てる状態にしてから走らせる**。ブリッジを`Ctrl-C`で
  落とすだけでは止まらない場合がある(機体が最後の指令のまま歩き続ける恐れがある)
- 無線非常停止(ハードウェア)は`cmd_vel_safety`のスコープ外。緊急時は実機の物理停止手段
  (リモコン・電源ボタン等)を使う準備をしておく。**リモコンが最後の砦**なので、
  リモコンの操作権をAPIに渡す設定(`UseRemoteCommandFromApi`)では走らせないこと
- **`cmd_vel_safety`の既定上限(`max_linear_x=1.0`, `max_angular_z=1.0`)は実機には高すぎる。**
  実機で詰めた実績値は `max_linear_x=0.22` / `max_linear_y=0.18` / `max_angular_z=0.45`
  (2026-09-02)。上のコマンド例のように明示的に下げてから使う
- **ただし0.15m/s程度を下回る上限にしてはいけない。** Go2の歩容はそこが下限で、それ未満は
  胴体が揺れるだけで前に進まない。「まず低速から」と0.1m/s以下で試すと「動かない」と誤認する

### 4. 顎3D LiDARの搭載位置キャリブレーション(Issue #4・C3)

**目的**: Gazebo側の`chin_lidar_joint`(`external/go2_ros2_sim_py/go2_description/xacro/robot.xacro`)は
現状、実機の搭載位置を測っていない**仮値**(`xyz="0.29 0.0 -0.06" rpy="0 0.35 0"`)。
このままだと自己位置推定(顎LiDAR点群→2D LaserScan→AMCL、自M2)の精度評価がsimでしか
意味を持たない。実機で1回測ってしまえば以降ずっと使える値になるので、接続当日にまとめて片付ける。

歩行確認(上記1〜3)が終わった後、ロボットを`Damp`か起立静止状態にしてから行う
(測定中に動かれると危ないので、`go2_sport_client 5`=StandDownか`1`=Dampで一旦力を抜くとよい)。

#### 4-1. 実測(必須・道具だけでできる。所要10分程度)

現在の仮値`xyz="0.29 0.0 -0.06" rpy="0 0.35 0"`は、**未使用の実機CAD由来URDF**
(`go2_description/urdf/go2_description.urdf`)にある`Head_lower`という実在のリンク
(base_link相対で x=0.293m前方, z=-0.06m下方)の値を流用したもの。`Head_lower`は
実機の頭部ユニット前面下側にある丸みを帯びた出っ張り(chin bump)に対応する、
**実物で指させる基準点**なので、`base_link`原点そのものを探す必要はなく、
この出っ張りからの相対位置さえ測れば良い。

1. [ ] ロボットの頭部ユニット前面・下側にある丸い出っ張り(`Head_lower`、通常カメラ類の
       下あたり)を確認し、その中心に印(マスキングテープ等)を付ける
2. [ ] その印から、実際に取り付けられている顎LiDARセンサの光学中心(レンズ中心・
       銘板に印がなければ筐体/マウント金具の幾何中心で代用)までの相対位置をメジャー/
       ノギスで測る。ROSの座標系(前方+x、左+y、上+z)に合わせて符号を付ける:
       - [ ] Δx(前後、前が+): _____ m
       - [ ] Δy(左右、左が+): _____ m
       - [ ] Δz(上下、上が+): _____ m
3. [ ] 最終的なxyz(xacroに書く値)を計算する:
       ```
       x = 0.293 + Δx
       y = 0.0   + Δy
       z = -0.06 + Δz
       ```
4. [ ] 下向きピッチ角を直接測る(ここが現状最も根拠のない値=0.35rad/20°の当て推量なので重要):
       - スマホの水平器/傾斜計アプリを、まず水平だとわかっている面
         (床に置いた状態のロボット本体上面など)に当てて0°表示になることを確認(アプリのキャリブレーション)
       - 同じアプリを顎LiDARセンサの前面(走査面/光軸に垂直な面)に当てて傾斜角を読む
       - [ ] 測定値: _____ °  →  ラジアンに変換: `rad = deg × π / 180` = _____ rad
       - 目安: 現状の仮値20°から大きく外れていないか(0°に近い・90°に近い等)は
         測る面を間違えていないかのサニティチェックになる
5. [ ] ロール角(左右の傾き)も同じアプリを90°回転させて当て、ほぼ0°であることを確認
       (左右非対称に傾いて取り付けられていないかの確認。通常は0°のはず)

この時点で`chin_lidar_joint`の`origin`に書く値が揃う:
`xyz="<x> <y> <z>" rpy="<roll_rad> <pitch_rad> 0"`

#### 4-2. 点群を見ながらの検証(顎LiDAR実機のROS2ドライバが動いている場合)

4-1の実測値は「だいたい合っている」レベルなので、可能なら点群を目で見て追い込む。

**(2026-09-02追記)**: 当初「顎LiDAR実機のROS2ドライバ(ベンダーSDK)が別途必要」という
前提で書いていたが、`unitree_ros2/README.md`を確認したところ**`unitree_ros2`自体が
実機接続時に標準で`/utlidar/cloud`(`PointCloud2`、frame_id: `utlidar_lidar`)を配信する**
とわかった(READMEの「visualizing robot lidar data」節)。つまり別途ベンダーSDKを
用意しなくても、DDS接続(上記1〜2)ができた時点でこの4-2はそのまま実行できる見込み。
念のため当日`ros2 topic echo --no-arr /utlidar/cloud`で実際に配信されているか確認すること。

1. [ ] 平らな床の上でロボットを、平らな壁に正対させて静止させる。ロボットの適当な基準点
       (胴体の前端など)から壁までの距離を正確に測っておく(例: 2.00m)
2. [ ] 4-1で出した値を`chin_lidar_joint`の`origin`に一旦入れるか、素早く試行錯誤したい場合は
       ```bash
       ros2 run tf2_ros static_transform_publisher \
         --x <x> --y <y> --z <z> \
         --roll <roll_rad> --pitch <pitch_rad> --yaw 0 \
         --frame-id base_link --child-frame-id utlidar_lidar
       ```
       で暫定TFを直接流す(URDFを毎回編集・再ビルドしなくて済む。名前付き引数なら
       roll/pitch/yawの並び順を間違えにくい)。**child-frame-idは`/utlidar/cloud`の
       実際のframe_id(`utlidar_lidar`)に合わせること**(以前の版は`chin_lidar_frame`と
       書いていたが、それだと自動変換(height_slice_viz等)が参照するframe_idと一致せず
       このTFが使われない)
3. [ ] `robot_state_publisher`(またはstatic_transform_publisher)+実機LiDARの点群を、
       RViz2でFixed Frameを`base_link`にして表示する
4. [ ] 確認する2点:
       - [ ] 床が水平でまっすぐ映っているか(傾いて映る場合はpitchがずれている。
             床が奥に向かって下るように見えたらpitchを緩め、上るように見えたらpitchを増やす方向)
       - [ ] 正面の壁が、測った距離(例: 2.00m)どおり・垂直に映っているか
             (近くor遠くに映る、斜めに映る場合はxyzがずれている)
5. [ ] ズレていれば値を微調整して2〜4を繰り返す。床が水平・壁が正しい距離に見えれば収束

#### 4-3. 結果の記録

- [ ] 最終的な値を`external/go2_ros2_sim_py/go2_description/xacro/robot.xacro`の
      `chin_lidar_joint`の`origin`に反映し、「仮値」コメントを実測日・方法に更新する
      (このファイルはフォーク側なので変更してよい)
- [ ] センサの実機型番・仕様(垂直chFOV等)が分かれば、`gazebo.xacro`の`chin_lidar`
      センサ定義(現状16ch・±15°も仮値)も合わせて更新する(Issue #4)
- [ ] 測定値・方法をIssue #4にコメントしておく

---

### 5. クソ雑map作成(実機M1/M2、Issue #10・#13・#67向け、2026-09-02追加)

**2026-09-04、実機で slam_toolbox が地図を出すところまで確認した。**
静止スキャン1枚から 3.35 x 19.35m の地図が立ち、TF `map -> base_link` も解決する。
歩かせながらの地図成長・保存はまだ実施していない。

起動は `./docker/driver/real_up.sh` にまとめてある(順序・上書き・自己検定込み)。
`--motion` を付けない限り機体が動く経路は起動しない。

```bash
./docker/driver/real_up.sh            # 観測系のみ(機体は動かない)
./docker/driver/real_up.sh --motion   # 走行系も(cmd_vel_safety + cmd_velブリッジ)
./docker/driver/real_up.sh status     # 何が動いているか
./docker/driver/real_up.sh down       # 全部止める
```

- [x] `ros2 topic echo /sportmodestate`が実データで届くことを確認(2026-09-04)
- [x] `state_to_odom_imu_node`が実データでOdometry/Imuを出すことを確認(295Hz、2026-09-04)
- [x] slam_toolboxが`/go2_localization/map`を配信することを確認(2026-09-04)
- [ ] RViz2(Fixed Frame: map)で地図が広がっていくのを見ながら歩かせる
- [ ] `ros2 run nav2_map_server map_saver_cli -f <保存先> --ros-args -p save_map_timeout:=5.0
      -r map:=/go2_localization/map` で保存

#### 実機でしか出なかった問題2つ(どちらも黙って壊れる)

**1. 機体の時計が開発PCより1109.7秒(約18.5分)遅れている。**
`/utlidar/cloud`のstampは機体時計、`state_to_odom_imu_node`のOdometry/Imuは
開発PCの時計(そのdocstringに書かれている設計判断どおり)。混ざると
slam_toolboxがスキャン時刻でodom->base_linkを引けず**全スキャンを捨てる**:

```
Message Filter dropping message: frame 'base_link' at time 1788498317.243
for reason 'the timestamp on the message is earlier than all the data in the transform cache'
```

機体の22/tcpは閉じておりログインできないので時刻同期はできない。
`go2_sport_bridge utlidar_cloud_restamp_node`で受信時に打ち直して中継する。
オフセットは40秒間で安定していたが、機体時計が跳ねても壊れないよう
「オフセットを引く」のではなく`now()`で打ち直している。

**2. `mapping_real.launch.py`のままでは床を障害物として地図に焼く。**
`height_slice_viz`の`cloud_in`が`/utlidar/cloud`に直結されているが、
床除去の理論距離計算がLiDAR搭載位置TFに依存しており、そのTFは
`static_tf_real.launch.py`のsim仮値(`pitch=0.35rad`)のままだったため。

実機には**`/utlidar/cloud_base`(ファームウェアが`base_link`座標系で配信、
frame_id: `base_link`)**があり、これを使えば**未実測の搭載位置TFを迂回できる**。
あわせて床の高さも実測すると`base_link`相対で**z≈-0.35**(設定値は
cafe_world由来の-0.27)だった。両方直した効果:

| | 修正前 | 修正後 |
|---|---|---|
| 有効ビーム | 241本 中央値0.60m | 107本 中央値1.82m |
| 1m未満の点 | 81.3%(=床) | **0%** |

`real_up.sh`はslam_toolboxを上げる前にこの比率を測り、30%を超えたら
**slam_toolboxを起動せずに中断する**(地図に焼き付いてから気づくのを防ぐため)。

**この2つの上書きは`go2_localization`側(担当が別)なので、起動時オプションで
渡しているだけでファイルには入れていない。** 恒久対応は要相談:
`height_slice_viz_real.launch.py`に入力トピックと`floor_z`のlaunch引数を足すのが素直。

#### その他、この節で判明したこと

- `/utlidar/cloud`: `sensor_msgs/PointCloud2`、frame_id `utlidar_lidar`、
  15.4Hz、1スキャン4142点。README記載どおりだった
- `/utlidar/cloud_base`: 同型で frame_id `base_link`、1スキャン1300〜1450点
- `/utlidar/cloud_deskewed`: frame_id が `odom`。EKFの`odom`と名前が衝突するので注意
- `/go2_localization/chin_lidar_scan`のQoSは**BEST_EFFORT**。既定QoS(RELIABLE)で
  購読すると`incompatible QoS`で1通も受け取れない
- slam_toolboxの`minimum laser range setting (0.0 m) exceeds the capabilities of
  the used Lidar (0.4 m)`は`range_min: 0.4`(脚のノイズ除去、Issue #26)由来で想定どおり

### 未確認・当日確認が必要な事項

- [x] Go2実機のIPアドレス → `192.168.123.161`(2026-09-04確認)
- [x] ホストのファイアウォールがマルチキャストDDS探索をブロックしないか
      → `GO2_NIC=enp2s0` で実機トピックが見えたためブロックされていない(2026-09-04確認)
- [x] Sport Mode APIの利用に純正アプリ側での事前操作が必要かどうか
      → **必要**。Unitree Goアプリで運動モードを「通常」にしないと、Moveは受理されるのに
      脚が出ない(2026-09-02実測)。APIからは切り替えられない
- [x] 顎LiDAR実機のROS2ドライバ(ベンダーSDK)が既に用意されているか
      → 不要。`/utlidar/cloud` がDDS接続だけで配信されていることを確認(2026-09-04)
