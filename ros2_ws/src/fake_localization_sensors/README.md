# fake_localization_sensors — 自己位置推定検証用のダミー Odometry/IMU パブリッシャ

Gazebo・実機なしでも `robot_localization`(EKF)の設定・疎通確認ができるように、
`nav_msgs/Odometry` と `sensor_msgs/Imu` を一定周期で publish するだけの最小 rclpy ノードです。

## 使い方

### ビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select fake_localization_sensors
source install/setup.bash
```

### 起動(最小)

```bash
ros2 run fake_localization_sensors fake_odom_imu_node
```

実行可能名(entry point)は `fake_odom_imu_node`、ノード名も `fake_odom_imu_node` です。
起動すると `FakeOdomImuNode has been started.` がログに出ます。

### 出力の確認

```bash
# 別ターミナルで(ワークスペースを source 済みの状態で)
ros2 topic echo /odom
ros2 topic echo /imu/data
ros2 topic hz /odom      # 既定では約 20Hz で流れる
```

### 代表的な引数

パラメータはコマンドライン引数(`-p 名前:=値`)で上書きできます。

```bash
# 直進(0.2 m/s)を 50Hz で出す
ros2 run fake_localization_sensors fake_odom_imu_node \
  -p publish_rate:=50.0 -p linear_velocity:=0.2 -p angular_velocity:=0.0

# 半径 1m 相当の円運動(v=0.2 m/s, ω=0.2 rad/s)
ros2 run fake_localization_sensors fake_odom_imu_node \
  -p linear_velocity:=0.2 -p angular_velocity:=0.2
```

トピック名を EKF 側の設定に合わせたい場合はリマップを使います。

```bash
ros2 run fake_localization_sensors fake_odom_imu_node \
  --ros-args -r odom:=/wheel/odometry -r imu/data:=/imu/data_raw
```

## 概要

自己位置推定の練習(自 M1 着手前の足慣らし、Issue #8)用に作られた、ダミーセンサノードです。
理想的な `Odometry` と `Imu` を一定周期で流し続けるだけで、購読はしません。
Gazebo や実機の実センサ(脚オドメトリ・IMU)がまだ無い段階でも、
`robot_localization` の EKF 設定を書いて動かし、フレーム・トピック・型の疎通を確認できるようにするのが狙いです。

船内 Go2 システムの 3 計画(自己位置推定 / 経路生成 / 経路追従)のうち
**自己位置推定** に属し、その中でも本番の推定器ではなく **sim・検証補助** の位置づけです。
つまり EKF に食わせる入力を模擬するテスト用ツールであり、推定そのものは行いません。

### ノード

| 項目 | 値 |
|------|-----|
| 実行可能名(entry point) | `fake_odom_imu_node` |
| ノード名 | `fake_odom_imu_node` |
| 購読トピック | なし |

### 入出力トピック

いずれもノード相対名で publish されます(リマップ・名前空間で変更可)。

| 種別 | トピック | 型 | フレーム |
|------|---------|-----|---------|
| 配信 | `odom` | `nav_msgs/Odometry` | `frame_id=odom` / `child_frame_id=base_link` |
| 配信 | `imu/data` | `sensor_msgs/Imu` | `frame_id=imu_link` |

### なぜ疑似センサが要るのか

EKF の設定(どのトピックのどの成分を融合するか、フレーム名、共分散の扱いなど)は、
実センサが揃う前でも先に書いて検証を始めたいことが多いです。
本ノードで既知の運動(直線 or 円)を理想値として流せば、
EKF の出力が入力どおりに追従するか、フレーム/トピックの配線が正しいかを
Gazebo・実機なしで単独確認できます。

## 詳細

### パラメータ

| 名前 | 型 | 既定値 | 意味 |
|------|-----|-------|------|
| `publish_rate` | double | `20.0` | publish 周期[Hz]。タイマ周期は `1.0 / publish_rate` 秒。 |
| `linear_velocity` | double | `0.2` | 並進速度[m/s]。ロボット前方(現在の yaw 方向)への速度。 |
| `angular_velocity` | double | `0.0` | 角速度[rad/s]。0 なら直線、非 0 なら等速円運動。 |

パラメータは起動時に一度だけ読み込まれます(実行中の動的変更には対応していません)。

### 内部ロジック(運動モデル)

タイマコールバックごとに、前回からの経過時間 `dt`(ナノ秒差 / 1e9)を求め、
等速円運動(`angular_velocity` が 0 なら直進)として姿勢を積分します。

```
yaw += angular_velocity * dt
x   += linear_velocity * cos(yaw) * dt
y   += linear_velocity * sin(yaw) * dt
```

初期状態は `x = y = yaw = 0`。時刻はすべて `get_clock().now()`(通常は system time)を使います。

### 生成する Odometry の中身

| フィールド | 設定値 |
|-----------|--------|
| `header.stamp` | 現在時刻 |
| `header.frame_id` | `odom` |
| `child_frame_id` | `base_link` |
| `pose.pose.position.x` / `.y` | 積分した `x` / `y`(z は 0) |
| `pose.pose.orientation.z` / `.w` | `sin(yaw/2)` / `cos(yaw/2)`(yaw を表す単位クォータニオン。x, y は 0) |
| `twist.twist.linear.x` | `linear_velocity`(そのまま) |
| `twist.twist.angular.z` | `angular_velocity`(そのまま) |

### 生成する IMU の中身

| フィールド | 設定値 |
|-----------|--------|
| `header.stamp` | 現在時刻 |
| `header.frame_id` | `imu_link` |
| `orientation.z` / `.w` | `sin(yaw/2)` / `cos(yaw/2)`(Odometry と同じ姿勢) |
| `angular_velocity.z` | `angular_velocity`(そのまま) |
| `linear_acceleration` | 未設定(既定の 0) |

### 注意点

- **ノイズは載せていません。** 理想値をそのまま出す最小構成です。
- **共分散(covariance)は設定していません。** すべて既定の 0 のまま publish されるため、
  EKF 側で共分散を参照する設定にしている場合は挙動に注意してください。
- IMU の `linear_acceleration`、および姿勢クォータニオンの x/y 成分は設定していません(0 のまま)。
- 2 次元平面(x, y, yaw)のみを模擬します。z・roll・pitch は常に 0 です。

## 動作確認結果(2026-07-14)

`odom`・`imu/data` とも指定周期で publish され、
`linear_velocity`/`angular_velocity` に応じて x/y/yaw が積分されることを確認済みです。
なお EKF 本体(`go2_localization`)は実際には Gazebo 実センサ(脚オドメトリ + IMU)を使う構成に進んだため、
本ノードは Gazebo・実機なしでの最初の疎通確認用途で役目を終えています。
