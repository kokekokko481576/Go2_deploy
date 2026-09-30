#!/usr/bin/env python3
"""Gazeboで**ヨー角の観測が真値とどれだけ食い違うか**を時系列で測る（sim限定・購読のみ）。

## なぜ要るか

真横への90度旋回（`final_heading:=right`）で、制御則の自己申告が「+86度回した」なのに
Gazeboの真値では約94度回っていた。3回とも同じ側に約9度ずれた（2026-09-08）。
ヨーの出所を odometry から IMU に替えても変わらなかったので、出所そのものではなく
**読み取りの遅れ**か**測るタイミング**を疑っている。それを見るための計測。

出るもの（0.1秒ごと）:
  真値      /gz_ground_truth/pose のヨー（Gazeboの物理演算。docker/sim/tools が配信）
  IMU       /robot1/imu_plugin/out のヨー
  odom      /robot1/odometry/filtered のヨー
  指令wz    /robot1/cmd_vel の angular.z
いずれも**開始時からの変化量**で出す（基準の取り方の違いを消すため）。

使い方（コンテナの中で）:
  ./docker/sim/tools/start_ground_truth.sh          # 真値publisherを先に起動
  docker exec -d go2-sim bash -c '. /opt/ros/jazzy/setup.bash && \
      python3 /sim_tools/sim_yaw_trace.py > /tmp/yaw.log 2>&1'
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class YawTrace(Node):

    def __init__(self):
        super().__init__('sim_yaw_trace')
        self.truth = self.imu = self.odom = None
        self.truth_xy = None    # 真の位置。**その場旋回で機体がどれだけ動くか**を見る
        self.marker = None      # カメラ座標系でのマーカー位置（橋渡しの出力）
        self.state = '-'
        self.wz = 0.0
        self.base = {}
        self.n = 0
        # センサ系はbest effortで出ていることがある。**reliableで待つと何も来ない**
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        def on_truth(m):
            self.truth = yaw_of(m.pose.orientation)
            self.truth_xy = (m.pose.position.x, m.pose.position.y)
        self.create_subscription(PoseStamped, '/gz_ground_truth/pose', on_truth, 10)
        self.create_subscription(PoseStamped, '/gz_ground_truth/pose', on_truth, sensor)
        self.create_subscription(Imu, '/robot1/imu_plugin/out',
                                 lambda m: setattr(self, 'imu', yaw_of(m.orientation)), 10)
        self.create_subscription(Imu, '/robot1/imu_plugin/out',
                                 lambda m: setattr(self, 'imu', yaw_of(m.orientation)), sensor)
        self.create_subscription(Odometry, '/robot1/odometry/filtered',
                                 lambda m: setattr(self, 'odom', yaw_of(m.pose.pose.orientation)), 10)
        self.create_subscription(Twist, '/robot1/cmd_vel',
                                 lambda m: setattr(self, 'wz', m.angular.z), 10)
        # 接近制御が「今どの区間か」と「マーカーをどこに見ているか」も並べる。
        # **真値と観測を同じ行に出さないと、どこで食い違ったかが分からない**
        self.create_subscription(String, '/marker_approach_node/state',
                                 lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(PoseStamped, 'marker_pose',
                                 lambda m: setattr(self, 'marker',
                                                   (m.pose.position.x, m.pose.position.y)), 10)
        self.create_timer(0.1, self.tick)
        print('t[s]   真値[度]  IMU[度]  odom[度]   指令wz  測定方位  移動[mm]  区間', flush=True)

    def tick(self):
        vals = {'truth': self.truth, 'imu': self.imu, 'odom': self.odom}
        for k, v in vals.items():
            if v is not None and k not in self.base:
                self.base[k] = v
        d = {k: (math.degrees(wrap(v - self.base[k])) if v is not None and k in self.base else float('nan'))
             for k, v in vals.items()}
        self.n += 1
        mb = (math.degrees(math.atan2(self.marker[1], self.marker[0]))
              if self.marker else float('nan'))
        if self.truth_xy and 'xy' not in self.base:
            self.base['xy'] = self.truth_xy
        moved = (1000.0 * math.hypot(self.truth_xy[0] - self.base['xy'][0],
                                     self.truth_xy[1] - self.base['xy'][1])
                 if self.truth_xy and 'xy' in self.base else float('nan'))
        print(f'{self.n * 0.1:5.1f} {d["truth"]:+9.2f} {d["imu"]:+8.2f} {d["odom"]:+8.2f} '
              f'{self.wz:+8.3f} {mb:+9.2f} {moved:9.0f}  {self.state}', flush=True)


def main():
    rclpy.init()
    node = YawTrace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
