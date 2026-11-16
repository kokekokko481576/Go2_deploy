#!/usr/bin/env python3
"""歩行速度と静止区間をリアルタイムに表示する(購読のみ、機体へは何も送らない)。2026-11-16

Classmethod記事(dev.classmethod.jp/articles/go2-patrol-slam_toolbox/)が推奨する
「0.2m/s程度のゆっくりしたペース、急旋回しない」を、勘ではなく数値で守れるようにする。
cmd_vel経由のMove APIには実用下限0.35m/s(2026-09-23実機実測、#75)があり0.2m/sを
システム的に強制できないため、リモコン操作のまま速度を可視化してフィードバックする設計にした。

静止監視(stop_monitor.py)の機能も統合している。/utlidar/robot_odom_restamped の
速度ノルムを見る。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class GaitMonitor(Node):
    def __init__(self, a):
        super().__init__('go2_gait_monitor')
        self.a = a
        self.stop_since = None
        self.last_msg_time = None
        self._last_speed = 0.0
        self._last_yaw_rate = 0.0
        self._prev_state = None
        self._last_announced = 0.0
        self._overspeed_since = None
        self._last_overspeed_announced = 0.0
        self.create_subscription(Odometry, a.topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)
        print(f'監視を開始します({a.topic}、購読のみ)。速度上限 {a.speed_limit}m/s、'
              f'静止しきい値 speed<{a.speed_th}m/s、静止警告 {a.warn}秒', flush=True)

    def on_odom(self, msg):
        now = time.time()
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        self.last_msg_time = now
        if speed < self.a.speed_th:
            if self.stop_since is None:
                self.stop_since = now
        else:
            self.stop_since = None
        self._last_speed = speed
        self._last_yaw_rate = msg.twist.twist.angular.z

    def tick(self):
        now = time.time()
        if self.last_msg_time is None or now - self.last_msg_time > 3.0:
            if self._prev_state != 'nodata':
                print(time.strftime('%H:%M:%S') + f'  {self.a.topic} が届いていません', flush=True)
                self._prev_state = 'nodata'
            return

        if self.stop_since is None:
            if self._prev_state != 'moving':
                print(f'{time.strftime("%H:%M:%S")}  走行中(再開)', flush=True)
                self._overspeed_since = None
                self._last_overspeed_announced = 0.0
            self._prev_state = 'moving'
            self._last_announced = 0.0
            self._tick_speed(now)
        else:
            dur = now - self.stop_since
            # 最初にしきい値を超えた時、その後はdur_stepごとにのみ知らせる(洪水防止)
            if dur >= self.a.warn and dur - self._last_announced >= self.a.dur_step:
                print(f'\a[注意] {time.strftime("%H:%M:%S")}  静止中 {dur:.1f}秒', flush=True)
                self._last_announced = dur
            self._prev_state = 'stopped'

    def _tick_speed(self, now):
        over = self._last_speed > self.a.speed_limit
        yaw_deg = math.degrees(self._last_yaw_rate)
        if over:
            if self._overspeed_since is None:
                self._overspeed_since = now
            dur = now - self._overspeed_since
            # 超過した瞬間、その後はdur_stepごとにのみ知らせる(洪水防止)
            if dur - self._last_overspeed_announced >= self.a.dur_step or self._last_overspeed_announced == 0.0:
                print(f'\a[超過] {time.strftime("%H:%M:%S")}  speed={self._last_speed:.2f}m/s '
                      f'(上限{self.a.speed_limit}m/s) yaw_rate={yaw_deg:+.0f}deg/s', flush=True)
                self._last_overspeed_announced = dur
        else:
            self._overspeed_since = None
            self._last_overspeed_announced = 0.0
            print(f'{time.strftime("%H:%M:%S")}  speed={self._last_speed:.2f}m/s '
                  f'yaw_rate={yaw_deg:+.0f}deg/s', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/utlidar/robot_odom_restamped')
    ap.add_argument('--speed-limit', type=float, default=0.2, help='これを超えたら警告[m/s](記事準拠)')
    ap.add_argument('--speed-th', type=float, default=0.03, help='これ未満を静止とみなす[m/s]')
    ap.add_argument('--warn', type=float, default=2.0, help='この秒数を超えたら静止警告[s]')
    ap.add_argument('--dur-step', type=float, default=5.0, help='警告後、再通知する間隔[s](洪水防止)')
    a = ap.parse_args()
    rclpy.init()
    node = GaitMonitor(a)
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
