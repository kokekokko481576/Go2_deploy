#!/usr/bin/env python3
"""接近が終わったあとの**真の**最終姿勢を、タグ基準で報告する（sim限定）。

制御則の自己申告ではなく Gazebo の真値で測る。移動フェーズが次段（アーム）に
どういう条件で機体を渡せるのかを数字で押さえるために使う。

出るもの: タグまでの距離 / 法線からの横ずれ / タグの見える方位（-90度＝右真横）。

使い方（コンテナの中で。真値publisherを先に起動しておくこと）:
  ./docker/sim/tools/start_ground_truth.sh
  docker exec go2-sim bash -c '. /opt/ros/jazzy/setup.bash && \
      python3 /sim_tools/sim_report_pose.py'
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# cafe.world の Apriltag36_11_00000 の位置と法線（法線は世界の +X 向き）
TAG = (-4.96, 1.5)
STANDOFF = 0.65
TARGET_BEARING = -90.0      # 右真横


def main():
    rclpy.init()
    n = Node('sim_report_pose')
    got = {}
    n.create_subscription(PoseStamped, '/gz_ground_truth/pose',
                          lambda m: got.__setitem__('p', m.pose), 10)
    t0 = time.time()
    while time.time() - t0 < 8 and 'p' not in got:
        rclpy.spin_once(n, timeout_sec=0.2)
    if 'p' not in got:
        print('真値が来ません（docker/sim/tools/start_ground_truth.sh を先に）')
        return 1
    p = got['p']
    x, y = p.position.x, p.position.y
    q = p.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)))
    d = math.hypot(x - TAG[0], y - TAG[1])
    lat = abs(y - TAG[1])
    b = (math.degrees(math.atan2(TAG[1] - y, TAG[0] - x)) - yaw + 180) % 360 - 180
    print(f'距離 {d:.3f}m({1000 * (d - STANDOFF):+.0f}mm)  '
          f'横ずれ {1000 * lat:3.0f}mm  '
          f'方位 {b:+.1f}度({b - TARGET_BEARING:+.1f}度)')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
