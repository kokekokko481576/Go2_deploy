# fake_localization_sensors

自己位置推定 練習(自M1着手前の足慣らし、Issue #8)用のダミーセンサノード。
`nav_msgs/Odometry`・`sensor_msgs/Imu`を一定周期でpublishするだけの最小rclpyノードで、
Gazebo・実機なしでも`robot_localization`(EKF)の設定・動作確認ができるようにする狙い。

## I/F

| 種別 | トピック | 型 |
|------|---------|-----|
| 配信 | `odom` | `nav_msgs/Odometry` |
| 配信 | `imu/data` | `sensor_msgs/Imu` |

パラメータ: `publish_rate`(既定20Hz)、`linear_velocity`(既定0.2 m/s)、`angular_velocity`(既定0 rad/s)。
等速円運動(`angular_velocity`が0なら直線)でx/y/yawを積分し、対応する`Odometry`/`Imu`を出す。
ノイズは載せていない(理想値をそのまま出す最小構成)。

## 使い方

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select fake_localization_sensors
source install/setup.bash
ros2 run fake_localization_sensors fake_odom_imu_node
```

```bash
# 別ターミナルで出力を確認
ros2 topic echo /odom
ros2 topic echo /imu/data
```

## 動作確認結果(2026-07-14)

`odom`・`imu/data`とも指定周期でpublishされ、`linear_velocity`/`angular_velocity`に応じて
x/y/yawが積分されることを確認。EKF自体(`go2_localization`)は実際にはGazebo実センサ
(脚オドメトリ+IMU)を使う構成に進んだため、本ノードはGazebo・実機無しでの最初の疎通確認
用途で役目を終えている。
