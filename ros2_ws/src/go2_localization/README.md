# go2_localization

自己位置推定 練習(自M1: EKF融合・自M2: AMCL稼働)用のbringupパッケージ。

このパッケージの本体は「既存パッケージ(`robot_localization`・`pointcloud_to_laserscan`・
`nav2_amcl`・`nav2_map_server`)の設定ファイル(yaml)+起動ファイル(launch.py)」だけで
構成されている。唯一の自作ノード`height_slice_viz`はAMCLの推定パイプラインには関与しない、
デバッグ可視化専用のもの(詳細は5節)。

## なぜこの設計になっているか(重要な前提)

simコンテナ(`external/go2_ros2_sim_py`)は**実は既に自前のNav2フルスタック
(EKF+AMCL+map_server+planner/controller等)を丸ごと起動している**
(`gazebo_multi_nav2_world.launch.py`)。つまり `map`→`odom`→`base_link` の
tf(座標変換)は、放っておいてもupstream側がすでに配信している。

自分たちの`go2_localization`をそのまま素朴に動かすと、同じフレーム名(`map`/`odom`/
`base_link`)に対して**2つのAMCL・2つのEKFがtfを配信し合って衝突する**。これを避けるため:

- 自分のEKF・AMCLは **`publish_tf`/`tf_broadcast`を`false`にし、tfは配信しない**
- 推定結果は`/go2_localization/...`という自分専用のトピックとしてのみ出力する
- 地図はupstreamの練習用ワールド(`cafe_world_map`)をそのまま拝借する(本番のマップは別途C4で整備する)

この方針により、実際のGazebo上の実センサ(IMU・顎LiDAR)を使いながら、upstream側の
動作とは衝突せずに自分のEKF/AMCLだけを独立して動かせる。

### ハマった実バグ: tfが名前空間付きだった

実際に動かして初めて分かったのだが、simの各ロボットは**グローバルな`/tf`ではなく
`/robot1/tf`・`/robot1/tf_static`という名前空間付きのトピックでtfを配信している**
(`gazebo_multi_nav2_world.launch.py`のremappings、`("/tf", "tf")`を
`namespace='robot1'`のノードに適用しているため)。

最初この remap を入れずに起動したところ、AMCLが
`Message Filter dropping message: ... the timestamp on the message is earlier than
all the data in the transform cache` を出し続けて`amcl_pose`が一切publishされなかった。
`ekf.launch.py`・`amcl.launch.py`に `('/tf', '/robot1/tf')` ・
`('/tf_static', '/robot1/tf_static')` のremapを足したら解消した。tf関連のノードが
「動いてはいるのに何も推定値が出ない」ときは、まずtfが本当に正しいトピックに
つながっているかを疑うとよい、という実例。

---

## 1. `config/ekf.yaml` + `launch/ekf.launch.py`(自M1: EKF融合)

### ekf.yaml

`robot_localization`パッケージの`ekf_node`(拡張カルマンフィルタ)への設定。

```yaml
/**:
  ekf_filter_node:
    ros__parameters:
      use_sim_time: true
```
- `/**:` は「どのノード名で起動されても、この`ekf_filter_node`という名前のノードには
  この設定を当てる」というワイルドカード(ROS2 yamlのお約束)
- `use_sim_time: true`: 壁時計ではなくGazeboの`/clock`トピックの時刻を使う。Gazebo内の
  シミュレーション時間とセンサのタイムスタンプを一致させるために必須(sim連携ノードは
  基本的に全部これをtrueにする)

```yaml
      frequency: 30.0
      two_d_mode: true
```
- `frequency`: 推定値を出力する頻度(Hz)。センサが届かなくても、この頻度では出力し続ける
- `two_d_mode`: true にすると z・roll・pitch を無視して平面(2D)だけ推定する。船内平地移動の
  自己位置推定はまず2Dで十分なため

```yaml
      publish_tf: false
      map_frame: map
      odom_frame: odom
      base_link_frame: base_link
      world_frame: odom
```
- `publish_tf: false`: 上の「なぜこの設計か」で説明した衝突回避のための最重要設定
- `world_frame: odom`: 「このEKFは連続的なローカル推定(odomフレーム基準)を出す」という指定。
  絶対位置(mapフレーム)の補正はAMCLの役目なので、EKFはodomフレームで完結させる

```yaml
      odom0: raw_odom_input
      odom0_config: [true,  true,  false,
                     false, false, true,
                     true,  true,  false,
                     false, false, true,
                     false, false, false]
      odom0_differential: false
      odom0_relative: false
```
- `odom0`: 1つ目のオドメトリ入力のトピック名(実際の解決は`launch`のremapで行う。
  下記参照)
- `odom0_config`: 15個の真偽値。`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch,
  vyaw, ax, ay, az]`の並びで「どの成分をこの入力から使うか」を選ぶ。ここでは
  x, y, yaw(向き), vx, vy, vyaw(速度)だけをtrueにしている(z・roll・pitchは
  two_d_modeで無視されるので使わない)

```yaml
      imu0: imu_plugin/out
      imu0_config: [false, false, false,
                    true,  true,  true,
                    false, false, false,
                    true,  true,  true,
                    true,  true,  true]
      imu0_differential: false
      imu0_relative: false
      imu0_remove_gravitational_acceleration: true
```
- `imu0`: IMU入力のトピック名
- `imu0_config`: IMUからはroll/pitch/yaw、各軸の角速度、各軸の加速度を使う(位置・速度は
  IMU単体からは分からないのでfalse)
- `imu0_remove_gravitational_acceleration: true`: 加速度計は静止していても重力加速度
  (約9.8m/s²)を検出し続けるので、それを差し引く設定。trueにしないと「静止しているのに
  常に加速している」と誤認識する

### ekf.launch.py

```python
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_config],
    remappings=[
        ('raw_odom_input', '/robot1/odometry/filtered'),
        ('imu_plugin/out', '/robot1/imu_plugin/out'),
        ('odometry/filtered', '/go2_localization/odometry/filtered'),
        ('/tf', '/robot1/tf'),
        ('/tf_static', '/robot1/tf_static'),
    ],
)
```
- `package`/`executable`: 自作ノードではなく、`robot_localization`パッケージに
  最初から入っている`ekf_node`という実行ファイルをそのまま使う
- `remappings`: ノードが内部で使っているトピック名を、実際のトピック名に付け替える設定。
  ここが一番ハマりやすいポイントなので詳しく説明する:
  - `raw_odom_input` → `/robot1/odometry/filtered`: yamlの`odom0`で設定した内部名
    `raw_odom_input`を、実際にはsim側が既に計算済みの`/robot1/odometry/filtered`に
    つなぐ。**本来は「生の脚オドメトリ」を使うべきだが、simでは生値が別トピックとして
    公開されていないため、upstream側の出力を「生オドメトリ」とみなして自分のEKFに
    もう一度かける、という練習用の割り切りをしている**(READMEの冒頭で断っている理由)
  - `odometry/filtered` → `/go2_localization/odometry/filtered`:
    `ekf_node`の出力トピック名は`odometry/filtered`に固定されている(yamlで変更不可)。
    このremapで自分専用の名前空間に逃がしている。**もし`raw_odom_input`側を
    remapせずに`odometry/filtered`という名前のままにしていたら、入力と出力が
    同じ名前になってしまい、どちらのremapが勝つか分からなくなる**(実際に
    upstream側のlaunchファイルでこの罠にハマっていそうな箇所があった)。
    入力側だけ内部名を`raw_odom_input`という別名にしておくことで、
    このあいまいさを避けている
  - `/tf`, `/tf_static` → `/robot1/tf`, `/robot1/tf_static`: 前述のハマりポイント

---

## 2. `config/pointcloud_to_laserscan.yaml` + `launch/pointcloud_to_laserscan.launch.py`(自M2前段)

### pointcloud_to_laserscan.yaml

顎3D LiDARの点群(`PointCloud2`)を、AMCLが読める2D `LaserScan`に変換するノードの設定。

```yaml
      target_frame: ""
      transform_tolerance: 0.05
```
- `target_frame: ""`: 空文字だと「入力のPointCloud2が持っているframe_idをそのまま使う」
  という意味になり、tf変換をしない(=このノード自体はtfに依存しない、シンプルな構成)

```yaml
      min_height: -0.05
      max_height: 0.05
```
- 3D点群のうち、この高さ範囲(顎LiDARの取り付け高さ±5cm)にある点だけを2Dスキャンとして
  採用する。3D点群を「輪切り」にして2Dにする、というイメージ

```yaml
      angle_min: -3.14159265
      angle_max: 3.14159265
      angle_increment: 0.0087
      range_min: 0.1
      range_max: 30.0
      use_inf: true
```
- 出力する2Dスキャンの角度範囲(-180°〜+180°、全周)と角度分解能(約0.5°刻み)、
  有効距離範囲
- `use_inf: true`: 障害物が無い方向は`inf`(無限遠)として表現する(AMCL・Nav2の標準的な扱い)

### pointcloud_to_laserscan.launch.py

```python
p2l_node = Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='pointcloud_to_laserscan',
    output='screen',
    parameters=[p2l_config],
    remappings=[
        ('cloud_in', '/robot1/chin_lidar/scan/points'),
        ('scan', '/go2_localization/chin_lidar_scan'),
    ],
)
```
- `cloud_in`: このノードの標準の入力トピック名。実際の顎LiDAR点群
  `/robot1/chin_lidar/scan/points`につなぐ
- `scan`: 標準の出力トピック名。`/go2_localization/chin_lidar_scan`という
  自分専用の名前に変えている(upstream側の2D LiDAR由来の`/robot1/scan`と混ざらないように)

---

## 3. `config/amcl.yaml` + `launch/amcl.launch.py`(自M2: AMCL稼働)

### amcl.yaml — `amcl`ブロック

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    tf_broadcast: false
```
- `tf_broadcast: false`: EKFの`publish_tf`と同じ理由。AMCLは本来`map`→`odom`のtfを
  配信するが、upstream側と衝突するので配信自体は止め、推定結果は`amcl_pose`トピック
  だけで受け取る

```yaml
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"
    scan_topic: "/go2_localization/chin_lidar_scan"
```
- `scan_topic`: ここは(EKFのodom0と違って)絶対パスの文字列をそのまま設定できるので、
  launchファイル側でremapしなくてもこの1行で完結する
- `odom_frame_id`: AMCLは内部で「`odom`→`base_link`のtfがどれだけ動いたか」を
  自分の動きモデルの入力として使う。この`odom`→`base_link`のtf自体は
  **upstream側のEKFが既に配信してくれているものをそのまま読みに行くだけ**
  (自分のEKFの出力ではないことに注意。自分のEKFはtf非配信なので、AMCLの動きモデルには
  直接関与しない。あくまで「自分のEKFはEKF単体の練習」「AMCLの動きモデルは
  upstreamの既存tfを間借り」という棲み分けになっている)

```yaml
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1〜alpha5: 0.2
```
- ロボットの動き方のモデル。差動二輪モデルを仮定し、動くたびにどれくらい姿勢が
  ばらつくと想定するか(alpha1〜5)を設定。値が大きいほど「動くと位置が不確かになりやすい」
  と仮定し、パーティクルが広がりやすくなる

```yaml
    max_particles: 2000
    min_particles: 500
```
- パーティクルフィルタが同時に保持する「自分はここにいるかもしれない」仮説の数。
  多いほど精度は上がるが計算負荷も増える

```yaml
    update_min_a: 0.2
    update_min_d: 0.25
```
- ロボットがこの距離(m)・角度(rad)以上動かないと、スキャンによる再推定を行わない
  (常に再計算すると重いので、動いた分だけ更新する)

### amcl.yaml — `map_server`ブロック

```yaml
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""
```
- `yaml_filename`はここでは空にしてあり、実際のパスは`amcl.launch.py`側で
  `get_package_share_directory`を使って絶対パスを組み立てて上書きしている
  (yamlに直接書くとインストール先ごとにパスが変わってしまうため)

### amcl.yaml — `lifecycle_manager_localization`ブロック

```yaml
lifecycle_manager_localization:
  ros__parameters:
    autostart: true
    node_names: ["map_server", "amcl"]
```
- Nav2の多くのノードは「lifecycle node」という、起動してもすぐには動かず
  `Configuring`→`Activating`という段階を踏んで有効化される仕組みになっている。
  `lifecycle_manager`はこの段階を自動で進めてくれる管理役。`node_names`に列挙した
  順で面倒を見る

### amcl.launch.py

```python
map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    parameters=[amcl_config, {'yaml_filename': map_yaml}],
    remappings=[('map', '/go2_localization/map')],
)
```
- `parameters=[amcl_config, {'yaml_filename': map_yaml}]`: リストの後の要素ほど
  優先される。yamlファイル本体を先に読み込ませ、そのあとで`yaml_filename`だけを
  実際の絶対パスで上書きしている
- `remappings=[('map', '/go2_localization/map')]`: map_serverの出力トピック名
  `map`を、自分専用の`/go2_localization/map`に変える

```python
amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    parameters=[amcl_config],
    remappings=[
        ('map', '/go2_localization/map'),
        ('amcl_pose', '/go2_localization/amcl_pose'),
        ('particlecloud', '/go2_localization/particlecloud'),
        ('/tf', '/robot1/tf'),
        ('/tf_static', '/robot1/tf_static'),
    ],
)
```
- `map`のremapをmap_server側と揃えることで、AMCLが自分のmap_serverから地図を
  受け取れるようにしている(揃っていないと「地図が来ない」状態になる)
- `amcl_pose`/`particlecloud`: AMCLの推定結果(平均姿勢/全パーティクル群)を、
  自分専用のトピック名で出す

```python
lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_localization',
    parameters=[amcl_config],
)
```
- yamlの`node_names: ["map_server", "amcl"]`に従って、この2つを順番に
  Configuring→Activatingしてくれる

---

## 4. `launch/localization.launch.py`(まとめ起動)

```python
ekf_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py'))
)
```
- 他のlaunchファイルを「部品」としてそのまま読み込んで合体させる仕組み。
  `ros2 launch go2_localization localization.launch.py`の1コマンドで
  EKF・pointcloud_to_laserscan・AMCL・`height_slice_viz`(5節)の4つが同時に起動する

---

## 5. `go2_localization/height_slice_viz.py` + `launch/height_slice_viz.launch.py`(デバッグ可視化)

### なぜ必要か(Issue #26関連)

`pointcloud_to_laserscan`の出力(`LaserScan`)をRViz2でそのまま見ようとしたところ、
この環境では**upstream本家の`/robot1/scan`(intensitiesも正常に入っている)を含めて
`LaserScan`表示が一切描画されない**現象が起きた(Position/Color Transformerが
両方とも空欄のまま)。データ自体は`ros2 topic echo`で正常と確認済みで、`PointCloud2`
表示は同じ環境で問題なく描画できていたため、原因はデータではなくこの環境の
`LaserScan`表示プラグイン側にあると判断し、深追いを避けて「`PointCloud2`のまま
見えるようにする」回避策に切り替えた。

### やっていること

`pointcloud_to_laserscan`と全く同じ「`target_frame`(=`base_link`)に変換してから
高さ(`min_height`〜`max_height`)で輪切りにする」処理を行うが、結果を`LaserScan`に
変換せず**`PointCloud2`のまま**`/go2_localization/chin_lidar_scan_points`に出力する。
AMCL本体が使う`pointcloud_to_laserscan`のパイプラインには一切手を加えず、
可視化専用の経路を並行して追加しただけの位置づけ。

- `tf2_ros.Buffer`で`target_frame`への変換(平行移動+回転)を取得
- 生の点群は`intensity`・`ring`など`x,y,z`以外のフィールドも持っており、
  `tf2_sensor_msgs.do_transform_cloud`にそのまま渡すと出力側の型組み立てで
  `AssertionError`になった(フィールド構成が混在するとうまく扱えない模様)。
  そのため`x,y,z`だけを`sensor_msgs_py.point_cloud2.read_points`で抜き出し、
  クォータニオン→回転行列に変換した上でNumPyで手動変換している
- 高さフィルタ後の点群は`create_cloud_xyz32`で組み立て直す
- `min_height`/`max_height`は`height_slice_viz.launch.py`が`pointcloud_to_laserscan.yaml`
  を直接読んで使う(手動同期はしていない)。`pointcloud_to_laserscan.yaml`を編集して
  どちらのlaunchも起動し直せば、両方に同じ値が反映される

### 動かし方

`localization.launch.py`に組み込み済みなので、単体では起動しない(単体起動したい場合のみ
`ros2 launch go2_localization height_slice_viz.launch.py`)。

RVizで`/go2_localization/chin_lidar_scan_points`を`PointCloud2`として追加すれば見える。

## 動かし方

devコンテナ + simコンテナの両方を起動した状態で:

```bash
# devコンテナ内
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_localization
source install/setup.bash
ros2 launch go2_localization localization.launch.py
```

別ターミナルで値を確認:

```bash
ros2 topic echo /go2_localization/odometry/filtered   # 自分のEKFの推定値
ros2 topic hz /go2_localization/chin_lidar_scan        # 顎LiDAR→2Dスキャン変換(約10Hz)
ros2 topic echo /go2_localization/amcl_pose            # AMCLの推定姿勢(mapフレーム)
```

RViz2で見る場合は、Fixed Frameを`map`にして、`/go2_localization/chin_lidar_scan_points`
(`PointCloud2`、5節)・`/go2_localization/particlecloud`(PoseArray)・`/go2_localization/amcl_pose`
(PoseWithCovarianceStamped)を追加すると、パーティクル雲や推定姿勢が見える
(`chin_lidar_scan`は`LaserScan`型だが、この環境ではRViz2のLaserScan表示自体が
描画できない不具合があるため使わないこと。5節参照)。

### テレオペで実際に歩かせる(自M1のドリフト計測・自M2の収束確認で使う)

`teleop_twist_keyboard`でsimコンテナ側の`/robot1/cmd_vel`に直接指令を送る
(`cmd_vel_safety`のクランプ・ウォッチドッグを経由するルートは未結線・未検証、
`cmd_vel_safety/README.md`参照)。

```bash
docker exec -it go2-sim bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

`u/i/o/j/k/l/m/,/.`で並進・旋回、`k`で停止。ウィンドウ(teleop_twist_keyboardを
実行しているターミナル)にフォーカスがないとキー入力を拾わないので注意。

## 動作確認結果(2026-07-13)

- devコンテナ→simコンテナ(別ホスト名・別ROS2ディストロ、cyclonedds経由)で、
  `/robot1/odometry/filtered`・`/robot1/imu_plugin/out`・`/robot1/chin_lidar/scan/points`が
  実際に見えることを確認
- `/go2_localization/chin_lidar_scan`が約10Hzで配信されることを確認
- `/go2_localization/odometry/filtered`(自分のEKF出力)が実データで配信されることを確認
- `/go2_localization/amcl_pose`が実データ(frame_id: map)で配信されることを確認。
  起動直後に1回だけ tf キャッシュ待ちのdropが出るが自然に解消する

## 未実施・既知の注意点

- ロボットが静止した状態でのみ確認。実際に歩かせながらの精度・収束の確認はこれから
  (issue #10のドリフト計測、issue #13の地図比較へ続く)
- Humble(dev)⇔Jazzy(sim)間のCycloneDDS警告(`invalid data size`/
  `string data is not null-terminated`)が出るが、既知の相性問題として
  issue #7で追跡中。値そのものは正しく届いている
- `odom0`にupstream側の`odometry/filtered`を「生オドメトリ」とみなして使っている点は
  本番の実機構成(脚オドメトリ+IMUを直接EKFに入れる)とは異なる、あくまでGazebo練習用の
  割り切り
