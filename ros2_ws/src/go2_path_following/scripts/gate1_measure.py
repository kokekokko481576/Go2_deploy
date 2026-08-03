#!/usr/bin/env python3
"""GATE1定量計測 (Issue #36): 到達成功率・部材正対精度・ゴール姿勢誤差を実測する。

前提: go2_localization・go2_path_following一式が起動済みで、simは
SIM_ENABLE_NAV2=false で起動していること
(go2_path_following/README.md「GATE1計測時のトピック確認」参照)。

使い方(devコンテナ内、フェーズB既定):
  ./gate1_measure.py [trials] --ros-args -r /tf:=/go2_localization/tf \
      -r /tf_static:=/robot1/tf_static

結果は scripts/results/gate1_<timestamp>.csv に1試行1行で書き出す。
集計は gate1_analyze.py <csv> で行う。
"""
import argparse
import csv
import math
import random
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import (ConnectivityException, ExtrapolationException,
                      LookupException, TransformListener)
from tf2_ros.buffer import Buffer

# controller_server.yaml の general_goal_checker と合わせる(README参照)
XY_TOLERANCE_M = 0.25
YAW_TOLERANCE_RAD = 0.2

GOAL_X_RANGE = (1.0, 3.0)
GOAL_Y_RANGE = (-1.0, 1.0)
POLL_PERIOD_S = 0.5
TRIAL_TIMEOUT_S = 30.0
SETTLE_PAUSE_S = 2.0
# 誤検出防止: 許容誤差内に連続でこの回数入ったら到達とみなす(通過だけの一瞬を弾く)
CONSECUTIVE_OK_REQUIRED = 2


def yaw_to_quat_zw(yaw_rad):
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quat_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class Gate1Measure(Node):

    def __init__(self):
        super().__init__('gate1_measure')
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def send_goal(self, x, y, yaw_rad):
        qz, qw = yaw_to_quat_zw(yaw_rad)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        # 配送遅延対策(send_goal.shと同じ理由。Issue #7周辺の混在DDS遅延): 複数回配信する
        for _ in range(3):
            self._goal_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.1)

    def current_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x, t.y, quat_to_yaw(q.z, q.w)

    def wait_and_measure(self, goal_x, goal_y, goal_yaw, timeout_s):
        start = time.monotonic()
        last_pose = None
        consecutive_ok = 0
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=POLL_PERIOD_S)
            pose = self.current_pose()
            if pose is None:
                continue
            last_pose = pose
            xy_err = math.hypot(pose[0] - goal_x, pose[1] - goal_y)
            yaw_err = abs(angle_diff(pose[2], goal_yaw))
            if xy_err <= XY_TOLERANCE_M and yaw_err <= YAW_TOLERANCE_RAD:
                consecutive_ok += 1
                if consecutive_ok >= CONSECUTIVE_OK_REQUIRED:
                    return True, pose, xy_err, yaw_err, time.monotonic() - start
            else:
                consecutive_ok = 0

        elapsed = time.monotonic() - start
        if last_pose is None:
            return False, None, None, None, elapsed
        xy_err = math.hypot(last_pose[0] - goal_x, last_pose[1] - goal_y)
        yaw_err = abs(angle_diff(last_pose[2], goal_yaw))
        return False, last_pose, xy_err, yaw_err, elapsed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', nargs='?', type=int, default=50)
    parser.add_argument('--seed', type=int, default=None,
                         help='乱数シード(再現したい場合に指定)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    if args.seed is not None:
        random.seed(args.seed)

    rclpy.init(args=sys.argv)
    node = Gate1Measure()

    results_dir = Path(__file__).resolve().parent / 'results'
    results_dir.mkdir(exist_ok=True)
    csv_path = results_dir / f'gate1_{time.strftime("%Y%m%d_%H%M%S")}.csv'

    fieldnames = ['trial', 'goal_x', 'goal_y', 'goal_yaw_deg',
                  'final_x', 'final_y', 'final_yaw_deg',
                  'xy_error_m', 'yaw_error_deg', 'success', 'elapsed_s']

    node.get_logger().info(f'GATE1計測開始: {args.trials}試行, 結果={csv_path}')

    success_count = 0
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(1, args.trials + 1):
            goal_x = random.uniform(*GOAL_X_RANGE)
            goal_y = random.uniform(*GOAL_Y_RANGE)
            goal_yaw_deg = random.uniform(0.0, 360.0)
            goal_yaw = math.radians(goal_yaw_deg)

            node.get_logger().info(
                f'[{i}/{args.trials}] goal=({goal_x:.2f}, {goal_y:.2f}, {goal_yaw_deg:.1f}deg)')
            node.send_goal(goal_x, goal_y, goal_yaw)

            success, pose, xy_err, yaw_err, elapsed = node.wait_and_measure(
                goal_x, goal_y, goal_yaw, TRIAL_TIMEOUT_S)

            row = {
                'trial': i,
                'goal_x': round(goal_x, 3),
                'goal_y': round(goal_y, 3),
                'goal_yaw_deg': round(goal_yaw_deg, 1),
                'final_x': round(pose[0], 3) if pose else '',
                'final_y': round(pose[1], 3) if pose else '',
                'final_yaw_deg': round(math.degrees(pose[2]), 1) if pose else '',
                'xy_error_m': round(xy_err, 3) if xy_err is not None else '',
                'yaw_error_deg': round(math.degrees(yaw_err), 1) if yaw_err is not None else '',
                'success': int(success),
                'elapsed_s': round(elapsed, 1),
            }
            writer.writerow(row)
            f.flush()

            if pose is not None:
                status = 'OK' if success else 'FAIL'
                node.get_logger().info(
                    f'  -> {status} xy_err={xy_err:.3f}m '
                    f'yaw_err={math.degrees(yaw_err):.1f}deg ({elapsed:.1f}s)')
            else:
                node.get_logger().warn('  -> FAIL (TFが一度も取得できなかった)')
            success_count += int(success)

            time.sleep(SETTLE_PAUSE_S)

    rate = 100.0 * success_count / args.trials if args.trials else 0.0
    node.get_logger().info(
        f'=== 完了: {success_count}/{args.trials} 成功 ({rate:.1f}%) ===')
    node.get_logger().info(f'結果: {csv_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
