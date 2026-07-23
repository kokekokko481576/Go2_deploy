# go2_localization — 自己位置推定（EKF融合 + LiDAR→LaserScan→AMCL）

自己位置推定 練習（自M1: 脚オドメトリ+IMUのEKF融合 ／ 自M2: 顎LiDAR点群→2D LaserScan→AMCL稼働）
用のbringupパッケージ。

このパッケージの本体は「既存パッケージ（`robot_localization`・`pointcloud_to_laserscan`・
`nav2_amcl`・`nav2_map_server`）の設定ファイル（yaml）+起動ファイル（launch.py）」だけで
構成されている。唯一の自作ノード `height_slice_viz` はAMCLの推定パイプラインには関与しない、
デバッグ可視化専用のもの（詳細は「詳細 > height_slice_viz」節）。

---

## 使い方

### 前提

- **devコンテナ（ROS2 Humble、このパッケージを動かす側）と simコンテナ（Gazebo）の両方が
  起動している**こと。simが動いていないとセンサ（`/robot1/odometry/filtered`・
  `/robot1/imu_plugin/out`・`/robot1/chin_lidar/scan/points`）が届かず、EKFもAMCLも何も出さない。
- simコンテナは既に自前のNav2フルスタック（EKF+AMCL+map_server等）を起動している。
  本パッケージは名前空間を分離してそれと衝突せずに動く（理由は「概要 > 名前空間分離の方針」・
  「詳細 > 名前空間分離設計」参照）。

### ビルドと統合起動（まずこれ）

`localization.launch.py` 1本で EKF・pointcloud_to_laserscan・AMCL・height_slice_viz の
4つがまとめて起動する。

```bash
# devコンテナ内
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_localization
source install/setup.bash
ros2 launch go2_localization localization.launch.py
```

### 動作確認

別ターミナルで主要トピックの値を確認する（`echo` が最も信頼できる。`hz` は混在環境で
偽陰性を出すことがある）:

```bash
ros2 topic echo /go2_localization/odometry/filtered   # 自分のEKFの推定値
ros2 topic echo /go2_localization/chin_lidar_scan      # 顎LiDAR→2Dスキャン（約10Hz）
ros2 topic echo /go2_localization/amcl_pose            # AMCLの推定姿勢（mapフレーム）
```

### RViz2で見る

Fixed Frame を `map` にして、以下を追加すると、パーティクル雲や推定姿勢が見える:

- `/go2_localization/chin_lidar_scan_points`（`PointCloud2`、床除去済み。height_slice_viz出力）
- `/go2_localization/particlecloud`（`PoseArray`）
- `/go2_localization/amcl_pose`（`PoseWithCovarianceStamped`）

> **注意**: `/go2_localization/chin_lidar_scan` は `LaserScan` 型だが、この環境では
> RViz2のLaserScan表示自体が描画できない不具合があるため RViz では使わない
> （データ自体は `ros2 topic echo` で正常。詳細は「詳細 > height_slice_viz」節）。

### 個別launchの起動

統合起動で足りるが、単体でデバッグしたいときは個別にも起動できる:

```bash
ros2 launch go2_localization ekf.launch.py                      # 自M1: EKF融合のみ
ros2 launch go2_localization pointcloud_to_laserscan.launch.py  # 顎LiDAR→2Dスキャンのみ
ros2 launch go2_localization amcl.launch.py                     # 自M2: AMCL+map_serverのみ
ros2 launch go2_localization height_slice_viz.launch.py         # デバッグ可視化のみ
```

いずれの個別launchも引数は取らない（設定はすべて `config/*.yaml` 側）。
`height_slice_viz.launch.py` は `localization.launch.py` に組み込み済みなので、通常は単体起動不要。

### テレオペで実際に歩かせる（自M1のドリフト計測・自M2の収束確認で使う）

`teleop_twist_keyboard` で simコンテナ側の `/robot1/cmd_vel` に直接指令を送る
（`cmd_vel_safety` のクランプ・ウォッチドッグを経由するルートは未結線・未検証、
`cmd_vel_safety/README.md` 参照）。

```bash
docker exec -it go2-sim bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

`u/i/o/j/k/l/m/,/.` で並進・旋回、`k` で停止。teleop_twist_keyboard を実行しているターミナルに
フォーカスがないとキー入力を拾わないので注意。

### 初期姿勢がズレたときの復旧（Issue #27）

localization起動**前**にteleop等でロボットを動かすと、その後起動しても自己位置が
めちゃくちゃになる。AMCLは起動時に「map原点にいる」固定値でパーティクルを撒く
（`amcl.yaml` の `set_initial_pose: true` / `initial_pose: {0,0,0}`）ため、実際の位置が
原点でないと仮定が外れたまま誤収束する。**起動順で防げる**（localizationを先に起動してから
動かす）が、やってしまった後の復旧手段が以下の2つ。

**復旧1（推奨）: 正しい初期姿勢を与え直す。** ロボットの実位置（map座標）が分かるとき用。
Gazebo画面から目測するか、upstream側が動いていれば
`ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot1/tf` で読む:

```bash
~/ros2_ws/src/go2_localization/scripts/set_initial_pose.sh 1.0 1.9 139   # x y yaw(度)
# 引数なしで実行すると原点(0,0,0度)に戻す
```

内部的には `/go2_localization/initialpose`（`PoseWithCovarianceStamped`、frame_id: map）へ
publishしてAMCLに再指定させる。共分散はRVizの2D Pose Estimateと同じ既定値（xy 0.25 / yaw 0.0685）。
混在環境の配送遅延対策として `-w 1`（subscriberマッチ待ち）+ 3回送信にしてある（#7周辺症状）。
動作確認済み（2026-07-17）: 原点から(1.01, 1.92, 139°)へ動かした後にlocalizationを起動して
破綻を再現→本スクリプトで**誤差2cm程度まで即時復旧**した。

**復旧2（最後の手段）: 大域再自己位置推定。** 位置の見当が全くつかないとき用。
パーティクルを地図全体に撒き直し、動き回りながら収束を待つ:

```bash
~/ros2_ws/src/go2_localization/scripts/relocalize_global.sh
# 内部で /reinitialize_global_localization サービスを呼ぶ。
# その後teleopで動かし回り、ros2 topic echo /go2_localization/amcl_pose で収束を見る
```

撒き直しただけでは収束しない（静止したままでは寄らない）。実行後にteleopで動かすと
観測と合う場所にパーティクルが寄っていく（適応リカバリも有効化済み）。
**注意（実測）**: cafeワールドでは通路の対称性のため収束が不安定（2m付近まで寄った後、
対称な別仮説へ十数m飛ぶ挙動を確認）。確実に直したいなら復旧1を使うこと。この
「対称環境での誤収束」は自M3（#14）の評価項目そのもので、定量評価はそちらで行う。

---

## 概要

### サブシステムの全体像（自M1 / 自M2）

| 計画 | 役割 | 使う既存パッケージ | 出力 |
|------|------|--------------------|------|
| 自M1 | 脚オドメトリ+IMUのEKF融合（連続的なローカル推定 `odom→base_link`） | `robot_localization`（`ekf_node`） | `/go2_localization/odometry/filtered` + TF `odom→base_link` |
| 自M2 前段 | 顎3D LiDAR点群 → 2D `LaserScan` 変換 | `pointcloud_to_laserscan` | `/go2_localization/chin_lidar_scan` |
| 自M2 | 地図とスキャンのマッチングで絶対位置推定（`map→odom` 補正） | `nav2_amcl` / `nav2_map_server` | `/go2_localization/amcl_pose` + TF `map→odom` |
| 補助 | 床除去済み点群のデバッグ可視化（推定には非関与） | 自作 `height_slice_viz` | `/go2_localization/chin_lidar_scan_points` |

役割分担: EKF が `odom→base_link` の連続的なローカル推定を担い、AMCL が `map→odom` の
絶対位置補正を担う。両者を合わせて `map→odom→base_link` の完結したTFツリーができる。

### データフロー

```
[Gazebo sim / robot1 名前空間]
  /robot1/odometry/filtered ──┐
  /robot1/imu_plugin/out ─────┼──▶ ekf_filter_node ──▶ /go2_localization/odometry/filtered
                              │         └──▶ TF: odom→base_link  ┐
                              │                                   │
  /robot1/chin_lidar/scan/points ─┬─▶ pointcloud_to_laserscan ──▶ /go2_localization/chin_lidar_scan
                                  │      (target_frame=base_link,        (LaserScan)         │
                                  │       /robot1/tf を参照)                                  ▼
                                  │                                                    ┌──────────┐
  map_server ──▶ /go2_localization/map ─────────────────────────────────────────────▶│   amcl   │
                                  │                     TF: odom→base_link ───────────▶│          │
                                  │                                                    └────┬─────┘
                                  │           /go2_localization/amcl_pose  ◀─────────────────┤
                                  │           /go2_localization/particlecloud  ◀─────────────┤
                                  │           TF: map→odom  ◀───────────────────────────────┘
                                  │
                                  └─▶ height_slice_viz ──▶ /go2_localization/chin_lidar_scan_points
                                         (床除去・デバッグ可視化専用、PointCloud2)
```

EKF が配信する `odom→base_link` と AMCL が配信する `map→odom` は、どちらも
専用トピック `/go2_localization/tf` に乗る（upstreamの `/robot1/tf` とは分離）。

### 主要な入出力トピック

| 向き | トピック | 型 | 説明 |
|------|----------|----|------|
| 入力（sim） | `/robot1/odometry/filtered` | `nav_msgs/Odometry` | sim側EKF出力。生オドメトリの代わりに流用（後述） |
| 入力（sim） | `/robot1/imu_plugin/out` | `sensor_msgs/Imu` | IMU |
| 入力（sim） | `/robot1/chin_lidar/scan/points` | `sensor_msgs/PointCloud2` | 顎3D LiDAR点群 |
| 入力（sim） | `/robot1/tf` `/robot1/tf_static` | `tf2_msgs/TFMessage` | upstreamのTF（静止TF、p2l/vizのbase_link変換元） |
| 出力 | `/go2_localization/odometry/filtered` | `nav_msgs/Odometry` | 自EKFの推定 |
| 出力 | `/go2_localization/chin_lidar_scan` | `sensor_msgs/LaserScan` | 顎LiDAR→2Dスキャン |
| 出力 | `/go2_localization/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL推定姿勢（mapフレーム） |
| 出力 | `/go2_localization/particlecloud` | `geometry_msgs/PoseArray` | 全パーティクル群 |
| 出力 | `/go2_localization/map` | `nav_msgs/OccupancyGrid` | 地図（map_server） |
| 出力 | `/go2_localization/chin_lidar_scan_points` | `sensor_msgs/PointCloud2` | 床除去済み点群（デバッグ） |
| 入力 | `/go2_localization/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初期姿勢の再指定（`set_initial_pose.sh`） |

### TF（座標変換）の分離

| TF | 配信元 | 乗るトピック |
|----|--------|--------------|
| `odom→base_link` | 自EKF（`ekf_filter_node`） | `/go2_localization/tf` |
| `map→odom` | 自AMCL | `/go2_localization/tf` |
| 静止TF（センサ位置等） | upstream `robot_state_publisher` | `/robot1/tf_static` |

### 名前空間分離の方針（重要な前提）

simコンテナ（`external/go2_ros2_sim_py`）は**実は既に自前のNav2フルスタック
（EKF+AMCL+map_server+planner/controller等）を丸ごと起動している**
（`gazebo_multi_nav2_world.launch.py`）。つまり `map→odom→base_link` のTFは、
放っておいてもupstream側がすでに配信している。

自分たちの `go2_localization` をそのまま素朴に動かすと、同じフレーム名（`map`/`odom`/
`base_link`）に対して**2つのAMCL・2つのEKFがtfを配信し合って衝突する**。これを避けるため、
自分のEKF・AMCLは `publish_tf`/`tf_broadcast` を `true` にしてTFを配信するが、出力先を
upstream本家の `/robot1/tf` とは別の **`/go2_localization/tf`** にlaunchでremapして名前空間を
分離している（静止TF=`/tf_static` は引き続きupstreamの robot_state_publisher から読む）。
推定結果も `/go2_localization/...` という自分専用のトピックとしてのみ出力する。
地図はupstreamの練習用ワールド（`cafe_world_map`）をそのまま拝借する（本番マップは別途C4）。

> **2026-07-14追記（Issue #21）**: 当初は衝突回避のため `publish_tf`/`tf_broadcast` を
> `false` にしTF自体を配信しない設計だったが、経路追従 `go2_path_following` の
> `controller_server` が実際の `map→odom→base_link` TFを必要とするため、上記の
> 名前空間分離方式に変更してTF配信を有効化した。設計判断の背景とハマった実バグは
> 「詳細 > 名前空間分離設計」に詳述する。

---

## 詳細

### 名前空間分離設計（なぜ `/go2_localization/tf` か）

この方針により、実際のGazebo上の実センサ（IMU・顎LiDAR）を使いながら、upstream側の
動作とは衝突せずに自分のEKF/AMCLだけを独立して動かせる。

#### ハマった実バグ: tfが名前空間付きだった

実際に動かして初めて分かったのだが、simの各ロボットは**グローバルな `/tf` ではなく
`/robot1/tf`・`/robot1/tf_static` という名前空間付きのトピックでtfを配信している**
（`gazebo_multi_nav2_world.launch.py` のremappings、`("/tf", "tf")` を
`namespace='robot1'` のノードに適用しているため）。

最初この remap を入れずに起動したところ、AMCLが
`Message Filter dropping message: ... the timestamp on the message is earlier than
all the data in the transform cache` を出し続けて `amcl_pose` が一切publishされなかった。
`ekf.launch.py`・`amcl.launch.py` に `('/tf', '/go2_localization/tf')`・
`('/tf_static', '/robot1/tf_static')` の類のremapを整えたら解消した。tf関連のノードが
「動いてはいるのに何も推定値が出ない」ときは、まずtfが本当に正しいトピックに
つながっているかを疑うとよい、という実例。

### EKF（`config/ekf.yaml` + `launch/ekf.launch.py`）— 自M1

`robot_localization` パッケージの `ekf_node`（拡張カルマンフィルタ）で、脚オドメトリ相当と
IMUを融合し `odom→base_link` の連続推定を出す。

#### ekf.yaml の主要パラメータ

```yaml
/**:
  ekf_filter_node:
    ros__parameters:
      use_sim_time: true
      frequency: 30.0
      sensor_timeout: 0.2
      two_d_mode: true
```
- `/**:` は「どのノード名で起動されても、この `ekf_filter_node` という名前のノードには
  この設定を当てる」というワイルドカード（ROS2 yamlのお約束）
- `use_sim_time: true`: 壁時計ではなくGazeboの `/clock` の時刻を使う。シミュレーション時間と
  センサのタイムスタンプを一致させるために必須（sim連携ノードは基本的に全部true）
- `frequency`: 推定値を出力する頻度（Hz）。センサが届かなくてもこの頻度で出力し続ける
- `sensor_timeout: 0.2`: 入力が0.2秒途切れたら、その入力なしで予測ステップだけ進める閾値
- `two_d_mode: true`: z・roll・pitch を無視して平面（2D）だけ推定する。船内平地移動の
  自己位置推定はまず2Dで十分なため

```yaml
      publish_tf: true
      map_frame: map
      odom_frame: odom
      base_link_frame: base_link
      world_frame: odom
```
- `publish_tf: true`: `odom→base_link` のTFを配信する設定。出力先は下記remapで
  upstreamと衝突しない専用トピック（`/go2_localization/tf`）に逃がす
- `world_frame: odom`: 「このEKFは連続的なローカル推定（odomフレーム基準）を出す」という指定。
  絶対位置（mapフレーム）の補正はAMCLの役目なので、EKFはodomフレームで完結させる

```yaml
      odom0: raw_odom_input
      odom0_config: [true,  true,  false,
                     false, false, true,
                     true,  true,  false,
                     false, false, true,
                     false, false, false]
      odom0_differential: false
      odom0_relative: false
      odom0_queue_size: 2
```
- `odom0`: 1つ目のオドメトリ入力のトピック名（実際の解決は launch のremapで行う）
- `odom0_config`: 15個の真偽値。`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch,
  vyaw, ax, ay, az]` の並びで「どの成分をこの入力から使うか」を選ぶ。ここでは
  x, y, yaw, vx, vy, vyaw だけをtrue（z・roll・pitchは two_d_mode で無視されるので使わない）

```yaml
      imu0: imu_plugin/out
      imu0_config: [false, false, false,
                    true,  true,  true,
                    false, false, false,
                    true,  true,  true,
                    true,  true,  true]
      imu0_differential: false
      imu0_relative: false
      imu0_queue_size: 5
      imu0_remove_gravitational_acceleration: true
```
- `imu0`: IMU入力のトピック名
- `imu0_config`: IMUからは roll/pitch/yaw、各軸の角速度、各軸の加速度を使う（位置・速度は
  IMU単体からは分からないのでfalse）
- `imu0_remove_gravitational_acceleration: true`: 加速度計は静止していても重力加速度
  （約9.8m/s²）を検出し続けるので差し引く設定。trueにしないと「静止しているのに常に加速して
  いる」と誤認識する

#### ekf.launch.py の remap（一番ハマりやすい所）

```python
ekf_node = Node(
    package='robot_localization', executable='ekf_node', name='ekf_filter_node',
    parameters=[ekf_config],
    remappings=[
        ('raw_odom_input', '/robot1/odometry/filtered'),
        ('imu_plugin/out', '/robot1/imu_plugin/out'),
        ('odometry/filtered', '/go2_localization/odometry/filtered'),
        ('/tf', '/go2_localization/tf'),
        ('/tf_static', '/robot1/tf_static'),
    ],
)
```
- `package`/`executable`: 自作ではなく `robot_localization` に最初から入っている `ekf_node` を使う
- `raw_odom_input` → `/robot1/odometry/filtered`: yamlの `odom0` に設定した内部名
  `raw_odom_input` を、実際にはsim側が既に計算済みの `/robot1/odometry/filtered` につなぐ。
  **本来は「生の脚オドメトリ」を使うべきだが、simでは生値が別トピックとして公開されていない
  ため、upstream側の出力を「生オドメトリ」とみなして自分のEKFにもう一度かける、という
  練習用の割り切りをしている**
- `odometry/filtered` → `/go2_localization/odometry/filtered`: `ekf_node` の出力トピック名は
  `odometry/filtered` に固定されている（yamlで変更不可）。このremapで自分専用の名前空間に逃がす。
  **入力側を `raw_odom_input` という別名にしておくことで、入力と出力が同名になって
  どちらのremapが勝つか分からなくなる、というあいまいさを避けている**
- `/tf` → `/go2_localization/tf`: 自分のEKFが配信する `odom→base_link` の出力先
- `/tf_static` → `/robot1/tf_static`: 静止TF（センサ位置等）は自分では配信せず、
  引き続きupstream（robot_state_publisher）を読みに行く

### pointcloud_to_laserscan（`config/…yaml` + `launch/…launch.py`）— 自M2前段

顎3D LiDARの点群（`PointCloud2`）を、AMCLが読める2D `LaserScan` に変換するノード。

```yaml
/**:
  pointcloud_to_laserscan:
    ros__parameters:
      use_sim_time: true
      target_frame: "base_link"
      transform_tolerance: 0.05

      min_height: -0.0
      max_height: 0.05

      angle_min: -3.14159265
      angle_max: 3.14159265
      angle_increment: 0.0087
      scan_time: 0.1
      range_min: 0.4
      range_max: 30.0
      use_inf: true
      concurrency_level: 1
```
- `target_frame: "base_link"`: **顎LiDARフレーム（`chin_lidar_frame`）は実機仮値で
  pitch=0.35rad（約20°）下向きに傾いている**。空文字（=センサ自身の傾いた座標系のまま高さで
  輪切り）にすると水平でない斜めの面で切ることになり、歩くたびに形が変わる不安定なスキャンに
  なる。`base_link` に変換してから輪切りにすることで傾きを補正する（Issue #26）。
  この設定のためこのノードはTFを必要とし、launch側で `/robot1/tf`・`/robot1/tf_static` を
  参照する
- `min_height` / `max_height`: `base_link` 相対でこの高さ帯（−0.0〜0.05m）にある点だけを
  2Dスキャンに採用する。顎LiDARは20°下向き+垂直FOV±15°で全ビームが水平から5〜35°下向きに
  しかならず、狭い帯だと遠方の壁が拾えず自分の脚・床の近距離ノイズばかりになっていた（Issue #26）
- `angle_min`〜`angle_increment`: 出力する2Dスキャンの角度範囲（−180°〜+180°、全周）と
  角度分解能（約0.5°刻み、`0.0087` rad）
- `range_min: 0.4`: 脚（0.13〜0.3m）の近距離ノイズを除外するため 0.1 から引き上げた（Issue #26）
- `use_inf: true`: 障害物が無い方向は `inf`（無限遠）として表現する（AMCL・Nav2の標準的な扱い）

```python
p2l_node = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    name='pointcloud_to_laserscan', parameters=[p2l_config],
    remappings=[
        ('cloud_in', '/robot1/chin_lidar/scan/points'),
        ('scan', '/go2_localization/chin_lidar_scan'),
        ('/tf', '/robot1/tf'),
        ('/tf_static', '/robot1/tf_static'),
    ],
)
```
- `cloud_in` → `/robot1/chin_lidar/scan/points`: 標準の入力トピック名を実際の顎LiDAR点群につなぐ
- `scan` → `/go2_localization/chin_lidar_scan`: 標準の出力トピック名を自分専用に変える
  （upstream側の2D LiDAR由来の `/robot1/scan` と混ざらないように）
- `/tf`・`/tf_static` → `/robot1/…`: `target_frame: base_link` への変換に必要なTFを
  upstreamから読む

### AMCL（`config/amcl.yaml` + `launch/amcl.launch.py`）— 自M2

`nav2_amcl`（パーティクルフィルタ）と `nav2_map_server`・`nav2_lifecycle_manager` の3点セット。

#### amcl.yaml — `amcl` ブロック

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    tf_broadcast: true
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"
    scan_topic: "/go2_localization/chin_lidar_scan"

    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1: 0.2   # alpha1〜alpha5 すべて 0.2

    laser_max_range: 30.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0
    max_beams: 60

    max_particles: 2000
    min_particles: 500
    update_min_a: 0.2
    update_min_d: 0.25
    resample_interval: 1
    transform_tolerance: 1.0

    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.1
    save_pose_rate: 0.5

    set_initial_pose: true
    initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
```
- `tf_broadcast: true`: AMCLは `map→odom` のTFを配信するが、下記remapで専用トピック
  （`/go2_localization/tf`）に逃がす。推定結果自体は `amcl_pose` トピックでも別途受け取れる
- `scan_topic`: ここは（EKFの `odom0` と違って）絶対パスの文字列をそのまま設定できるので、
  launch側でremapしなくてもこの1行で完結する
- `odom_frame_id`: AMCLは内部で「`odom`→`base_link` のtfがどれだけ動いたか」を動きモデルの
  入力に使う。下記remapで `/tf` を `/go2_localization/tf` に向けているため、この
  `odom→base_link` は**自分のEKFが配信しているもの**を読む。現在は自分のEKF・AMCLだけで
  完結したTFツリーになっている
- `robot_model_type` / `alpha1`〜`alpha5`: 差動二輪の動きモデル。動くたびにどれくらい姿勢が
  ばらつくと想定するか。値が大きいほどパーティクルが広がりやすい
- `laser_model_type: "likelihood_field"` / `max_beams: 60`: 尤度場モデルで、1スキャンから
  最大60ビームだけをサンプルして評価する（全ビーム使うより軽い）
- `max_particles` / `min_particles`: パーティクルフィルタが保持する「ここにいるかも」仮説の数
- `update_min_a` / `update_min_d`: ロボットがこの角度（rad）・距離（m）以上動かないと
  スキャン再推定を行わない（常時再計算は重いので動いた分だけ更新）
- `transform_tolerance: 1.0`: 配信するTFに与える有効期限（秒）の余裕。混在環境の遅延に対して
  大きめにとっている
- `recovery_alpha_slow: 0.001` / `recovery_alpha_fast: 0.1`: 適応リカバリ（Augmented MCL）。
  観測尤度の短期平均が長期平均を下回るとランダムパーティクルを注入し、初期姿勢の仮定が
  外れていても自力回復の芽を残す（Issue #27。0.0/0.0だと完全無効=一度誤収束すると戻れない）。
  値は Probabilistic Robotics / nav2ドキュメントの推奨値
- `set_initial_pose: true` / `initial_pose`: 起動時は「map原点にいる」前提でタイトに撒く。
  原点から動かした後に起動するとこの仮定が外れて破綻する（「使い方 > 初期姿勢の復旧」参照）

#### amcl.yaml — `map_server` ブロック

```yaml
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""
```
- `yaml_filename` はここでは空にし、実際のパスは `amcl.launch.py` 側で
  `get_package_share_directory` を使って絶対パスを組み立てて上書きする（yamlに直接書くと
  インストール先ごとにパスが変わってしまうため）

#### amcl.yaml — `lifecycle_manager_localization` ブロック

```yaml
lifecycle_manager_localization:
  ros__parameters:
    use_sim_time: true
    autostart: true
    node_names: ["map_server", "amcl"]
```
- Nav2の多くのノードは「lifecycle node」という、起動してもすぐには動かず
  `Configuring`→`Activating` の段階を踏んで有効化される仕組み。`lifecycle_manager` は
  この段階を自動で進める管理役。`node_names` に列挙した順で面倒を見る

#### amcl.launch.py の remap

```python
map_server_node = Node(
    package='nav2_map_server', executable='map_server', name='map_server',
    parameters=[amcl_config, {'yaml_filename': map_yaml}],
    remappings=[('map', '/go2_localization/map')],
)
amcl_node = Node(
    package='nav2_amcl', executable='amcl', name='amcl',
    parameters=[amcl_config],
    remappings=[
        ('map', '/go2_localization/map'),
        ('amcl_pose', '/go2_localization/amcl_pose'),
        ('particlecloud', '/go2_localization/particlecloud'),
        ('initialpose', '/go2_localization/initialpose'),
        ('/tf', '/go2_localization/tf'),
        ('/tf_static', '/robot1/tf_static'),
    ],
)
lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager', executable='lifecycle_manager',
    name='lifecycle_manager_localization', parameters=[amcl_config],
)
```
- `parameters=[amcl_config, {'yaml_filename': map_yaml}]`（map_server）: リストの後の要素ほど
  優先される。yaml本体を先に読み込ませ、そのあと `yaml_filename` だけを実際の絶対パスで上書きする
- `map` → `/go2_localization/map`: map_serverの出力とAMCLの入力を揃えることで、AMCLが自分の
  map_serverから地図を受け取れるようにする（揃っていないと「地図が来ない」状態になる）
- `amcl_pose` / `particlecloud` → `/go2_localization/…`: AMCLの推定結果（平均姿勢/全パーティクル
  群）を自分専用のトピック名で出す
- `initialpose` → `/go2_localization/initialpose`: 初期姿勢の再指定入力（Issue #27、
  `scripts/set_initial_pose.sh` がここへpublishする）。他トピックと同様に専用名前空間へ分離
- `/tf` → `/go2_localization/tf`: `odom→base_link` の入力（自分のEKFの出力）も `map→odom` の
  出力（AMCL自身）も、この同じ専用トピックに乗せる
- `/tf_static` → `/robot1/tf_static`: 静止TFは自分では配信せず、upstreamを読み続ける
- `lifecycle_manager`: yamlの `node_names: ["map_server", "amcl"]` に従い、この2つを順に
  Configuring→Activating してくれる

### 地図（`config/map/cafe_world_map.yaml`）

upstreamの練習用ワールド `cafe_world` の地図をそのまま拝借している（本番マップは別途C4で整備）。

```yaml
image: cafe_world_map.pgm
mode: trinary
resolution: 0.05          # 1ピクセル=5cm
origin: [-5.06, -11.1, 0] # 地図左下の原点(map座標)
negate: 0
occupied_thresh: 0.65     # これ以上の占有率のセルを障害物とみなす
free_thresh: 0.25         # これ以下を自由空間とみなす
```
- `map_server` が起動時にこのyaml（と隣の `cafe_world_map.pgm`）を読み、
  `/go2_localization/map`（`OccupancyGrid`）として配信する

### height_slice_viz（`go2_localization/height_slice_viz.py` + launch）— デバッグ可視化

#### なぜ必要か（Issue #26関連）

`pointcloud_to_laserscan` の出力（`LaserScan`）をRViz2でそのまま見ようとしたところ、
この環境では**upstream本家の `/robot1/scan`（intensitiesも正常）を含めて `LaserScan` 表示が
一切描画されない**現象が起きた（Position/Color Transformerが両方とも空欄のまま）。
データ自体は `ros2 topic echo` で正常と確認済みで、`PointCloud2` 表示は同じ環境で問題なく
描画できていたため、原因はデータではなくこの環境の `LaserScan` 表示プラグイン側にあると判断し、
深追いを避けて「`PointCloud2` のまま見えるようにする」回避策に切り替えた。

#### やっていること（床除去ロジック）

当初は `pointcloud_to_laserscan` と同じ「`target_frame`（=`base_link`）に変換してから
高さで輪切りにする」処理だったが、これだと**床と壁を原理的に区別できない**問題があった。
遠方の壁を拾うには低い高さ帯が必要だが、その帯には様々な距離で床（距離が変わってもほぼ同じ
高さにある水平面）も入り込んでしまい、高さだけでは「たまたま来た床」と「本当にある壁」を
区別できない。

そこで**レイごとの理論上の床到達距離との比較**に変更した。センサの搭載位置・角度から、各レイ
（センサ原点→その点への直線）が仮に何もない床（`target_frame` 相対で `floor_z` の高さ）まで
届いたとしたら何m先になるかを逆算し、実際の反射距離がそれより `floor_margin` 以上手前なら
「床の手前に何かある」=障害物、理論距離付近〜以遠なら床、と判定して床側を除去する。結果は
`LaserScan` に変換せず **`PointCloud2` のまま** `/go2_localization/chin_lidar_scan_points` に
出力する。AMCL本体が使う `pointcloud_to_laserscan` のパイプラインには一切手を加えず、
可視化専用の経路を並行して追加しただけの位置づけ（将来的に効果が確認できたらAMCL側の
`pointcloud_to_laserscan` をこの床除去ロジックを持つ自作ノードに置き換えることも検討）。

実装上のポイント（`height_slice_viz.py`）:
- `tf2_ros.Buffer` で `target_frame` への変換（平行移動+回転）を取得。平行移動成分がそのまま
  「`target_frame` から見たセンサ原点の位置」になる
- 生の点群は `intensity`・`ring` など `x,y,z` 以外のフィールドも持っており、
  `tf2_sensor_msgs.do_transform_cloud` にそのまま渡すと出力側の型組み立てで `AssertionError` に
  なった。そのため `x,y,z` だけを `point_cloud2.read_points` で抜き出し、クォータニオン→回転行列
  に変換した上でNumPyで手動変換している
- `skip_nans=True` は `inf`（検出なし）までは除かないため、`np.isfinite` で別途フィルタする
- 床除去後の点群は `create_cloud_xyz32` で組み立て直す

#### 調整パラメータ

`config/pointcloud_to_laserscan.yaml` の **`height_slice_viz:` ブロック**で調整する
（`pointcloud_to_laserscan` ノード本体とは別ブロック。同じyamlに同居させているだけ）:

| パラメータ | yamlの値 | ノード内デフォルト | 意味 |
|-----------|---------|--------------------|------|
| `target_frame` | `"base_link"` | `base_link` | 点群の変換先フレーム |
| `floor_z` | `-0.27` | `-0.3` | 床の高さ（`target_frame` 相対）。cafe_worldの平地で静止スキャンした実測（前方0.5〜2.0mで一貫してz=−0.26〜−0.28）から−0.27 |
| `floor_margin` | `0.15` | `0.1` | 理論上の床到達距離より、これ以上手前で反射していれば障害物とみなすマージン |
| `max_height` | `0.3` | `0.3` | 天井反射等を除く粗い上限 |

（yamlの値が優先。ノード内 `declare_parameter` のデフォルトは launch を介さず単体実行した場合の値）

---

## 動作確認結果（2026-07-13）

- devコンテナ→simコンテナ（別ホスト名・別ROS2ディストロ、cyclonedds経由）で、
  `/robot1/odometry/filtered`・`/robot1/imu_plugin/out`・`/robot1/chin_lidar/scan/points` が
  実際に見えることを確認
- `/go2_localization/chin_lidar_scan` が約10Hzで配信されることを確認
- `/go2_localization/odometry/filtered`（自分のEKF出力）が実データで配信されることを確認
- `/go2_localization/amcl_pose` が実データ（frame_id: map）で配信されることを確認。
  起動直後に1回だけ tf キャッシュ待ちのdropが出るが自然に解消する

## 未実施・既知の注意点

- ロボットが静止した状態でのみ確認。実際に歩かせながらの精度・収束の確認はこれから
  （issue #10のドリフト計測、issue #13の地図比較へ続く）
- Humble（dev）⇔Jazzy（sim）間のCycloneDDS警告（`invalid data size` /
  `string data is not null-terminated`）が出るが、既知の相性問題として issue #7で追跡中。
  値そのものは正しく届いている
- `odom0` にupstream側の `odometry/filtered` を「生オドメトリ」とみなして使っている点は、
  本番の実機構成（脚オドメトリ+IMUを直接EKFに入れる）とは異なる、あくまでGazebo練習用の割り切り
