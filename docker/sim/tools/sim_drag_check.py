#!/usr/bin/env python3
"""「その場旋回」で機体がどれだけ動くか、真値とオドメトリの両方で測る（sim限定）。

真横への90度旋回のあと、マーカーが真横から7〜11度ずれて止まる原因を追ったときに作った。
分かったこと（2026-09-08、Gazebo）: **その場旋回は その場ではない。** 80度回る間に
機体が117mm動く（90度なら約130mm）。0.6mの距離ではこれが方位11度に相当する。

制御則は旋回中のマーカー方位を推測航法で追って補正するが、**オドメトリがこの移動を
観測できていなければ補正のしようがない。** それを確かめるのがこのツール。

使い方（コンテナの中で。**機体が回る**）:
  ./docker/sim/tools/start_ground_truth.sh
  docker exec go2-sim bash -c '. /opt/ros/jazzy/setup.bash && \
      python3 /sim_tools/sim_drag_check.py'
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = Node('sim_drag_check')
    d = {}
    n.create_subscription(Odometry, '/robot1/odometry/filtered',
                          lambda m: d.__setitem__('odom', (m.pose.pose.position.x,
                                                           m.pose.pose.position.y,
                                                           yaw_of(m.pose.pose.orientation))), 10)
    n.create_subscription(PoseStamped, '/gz_ground_truth/pose',
                          lambda m: d.__setitem__('true', (m.pose.position.x,
                                                           m.pose.position.y,
                                                           yaw_of(m.pose.orientation))), 10)
    pub = n.create_publisher(Twist, '/robot1/cmd_vel', 10)

    t0 = time.time()
    while time.time() - t0 < 6 and len(d) < 2:
        rclpy.spin_once(n, timeout_sec=0.2)
    if len(d) < 2:
        print('真値かオドメトリが来ません（start_ground_truth.sh を先に実行）')
        return 1
    start = dict(d)

    tw = Twist()
    tw.angular.z = 0.35
    t0 = time.time()
    while time.time() - t0 < 6.0:
        pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)
    tw.angular.z = 0.0
    for _ in range(20):          # 止める。**simにはウォッチドッグが無い**
        pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)

    for k in ('true', 'odom'):
        a, b = start[k], d[k]
        moved = 1000.0 * math.hypot(b[0] - a[0], b[1] - a[1])
        turned = math.degrees((b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi)
        print(f'{k:5s}: 回った {turned:+7.1f}度   移動 {moved:6.0f} mm')
    print('※ 真値が動いていてオドメトリが動いていなければ、'
          '**引きずりはオドメトリから観測できない**＝推測航法では補正できない')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
