#!/usr/bin/env python3
"""接触無し対照実験 (Issue #54): 家具に絶対触れない直線往復のみで、
セッション内ドリフト蓄積(#36で確認)が接触無しでも発生するか切り分ける。

GATE1(#36)のゴール範囲(x:1-3m, y:-1-1m)は最寄りの家具table1(cafe.world内、
x=0.5, y=-1.6)からy方向に1.6m以上離れているため、y=0固定でx=1.0/3.0を
往復させれば家具に接触しない(cafe.world内の他の家具・壁はさらに遠い)。

前提: gate1_measure.pyと同じ(go2_localization・go2_path_following起動済み、
SIM_ENABLE_NAV2=false)。docker/sim/tools/start_ground_truth.shも起動しておくこと
(真値との比較がこの対照実験の目的そのものなので、gate1_measure.pyより必須度が高い)。

使い方(devコンテナ内、フェーズB既定):
  ./drift_control_measure.py [legs] --ros-args -r /tf:=/go2_localization/tf \
      -r /tf_static:=/robot1/tf_static

結果は scripts/results/drift_control_<timestamp>.csv に1レグ1行で書き出す。
gate1_measure.pyと同一のCSVスキーマなので gate1_analyze.py で集計できる
(往復に成功していれば到達成功率は100%近くになるはず。前半/後半でgt_xy_error_mの
平均が伸びていればドリフトは接触無関係に蓄積している=AMCL/EKFパラメータ側の問題、
伸びていなければGATE1の蓄積は接触起因が濃厚)。
"""
import argparse
import csv
import math
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

# controller_server.yaml の general_goal_checker と合わせる(gate1_measure.py参照)
XY_TOLERANCE_M = 0.25
YAW_TOLERANCE_RAD = 0.2

# 家具・壁から十分離れた直線(cafe.world: table1が(0.5,-1.6)で最も近い)
WAYPOINTS = [(1.0, 0.0, 0.0), (3.0, 0.0, math.pi)]
POLL_PERIOD_S = 0.5
TRIAL_TIMEOUT_S = 30.0
SETTLE_PAUSE_S = 2.0
CONSECUTIVE_OK_REQUIRED = 2


def yaw_to_quat_zw(yaw_rad):
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quat_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class DriftControlMeasure(Node):

    def __init__(self):
        super().__init__('drift_control_measure')
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_gt_pose = None
        self.create_subscription(
            PoseStamped, '/gz_ground_truth/pose', self._on_ground_truth, 10)

    def _on_ground_truth(self, msg):
        q = msg.pose.orientation
        self._latest_gt_pose = (
            msg.pose.position.x, msg.pose.position.y, quat_to_yaw(q.z, q.w))

    def has_ground_truth(self):
        return self._latest_gt_pose is not None

    def send_goal(self, x, y, yaw_rad):
        qz, qw = yaw_to_quat_zw(yaw_rad)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
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
        last_gt_pose = None
        consecutive_ok = 0
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=POLL_PERIOD_S)
            pose = self.current_pose()
            if self._latest_gt_pose is not None:
                last_gt_pose = self._latest_gt_pose
            if pose is None:
                continue
            last_pose = pose
            xy_err = math.hypot(pose[0] - goal_x, pose[1] - goal_y)
            yaw_err = abs(angle_diff(pose[2], goal_yaw))
            if xy_err <= XY_TOLERANCE_M and yaw_err <= YAW_TOLERANCE_RAD:
                consecutive_ok += 1
                if consecutive_ok >= CONSECUTIVE_OK_REQUIRED:
                    return True, pose, xy_err, yaw_err, time.monotonic() - start, last_gt_pose
            else:
                consecutive_ok = 0

        elapsed = time.monotonic() - start
        if last_pose is None:
            return False, None, None, None, elapsed, last_gt_pose
        xy_err = math.hypot(last_pose[0] - goal_x, last_pose[1] - goal_y)
        yaw_err = abs(angle_diff(last_pose[2], goal_yaw))
        return False, last_pose, xy_err, yaw_err, elapsed, last_gt_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('legs', nargs='?', type=int, default=50,
                         help='往復レグ数(gate1_measure.pyの試行数50に合わせるとGATE1と比較しやすい)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = DriftControlMeasure()

    results_dir = Path(__file__).resolve().parent / 'results'
    results_dir.mkdir(exist_ok=True)
    csv_path = results_dir / f'drift_control_{time.strftime("%Y%m%d_%H%M%S")}.csv'

    fieldnames = ['trial', 'goal_x', 'goal_y', 'goal_yaw_deg',
                  'final_x', 'final_y', 'final_yaw_deg',
                  'xy_error_m', 'yaw_error_deg', 'success', 'elapsed_s',
                  'gt_x', 'gt_y', 'gt_yaw_deg',
                  'gt_xy_error_m', 'gt_yaw_error_deg', 'gt_success']

    node.get_logger().info(f'接触無し対照実験開始: {args.legs}レグ, 結果={csv_path}')
    for _ in range(4):
        if node.has_ground_truth():
            break
        rclpy.spin_once(node, timeout_sec=0.5)
    if not node.has_ground_truth():
        node.get_logger().warn(
            '真値(/gz_ground_truth/pose)が来ていません。この対照実験は真値比較が目的なので、'
            'docker/sim/tools/start_ground_truth.shを起動してから再実行してください')

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(1, args.legs + 1):
            goal_x, goal_y, goal_yaw = WAYPOINTS[(i - 1) % len(WAYPOINTS)]
            goal_yaw_deg = math.degrees(goal_yaw)

            node.get_logger().info(
                f'[{i}/{args.legs}] goal=({goal_x:.2f}, {goal_y:.2f}, {goal_yaw_deg:.1f}deg)')
            node.send_goal(goal_x, goal_y, goal_yaw)

            success, pose, xy_err, yaw_err, elapsed, gt_pose = node.wait_and_measure(
                goal_x, goal_y, goal_yaw, TRIAL_TIMEOUT_S)

            gt_xy_err = gt_yaw_err = gt_success = None
            if gt_pose is not None:
                gt_xy_err = math.hypot(gt_pose[0] - goal_x, gt_pose[1] - goal_y)
                gt_yaw_err = abs(angle_diff(gt_pose[2], goal_yaw))
                gt_success = (gt_xy_err <= XY_TOLERANCE_M
                              and gt_yaw_err <= YAW_TOLERANCE_RAD)

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
                'gt_x': round(gt_pose[0], 3) if gt_pose else '',
                'gt_y': round(gt_pose[1], 3) if gt_pose else '',
                'gt_yaw_deg': round(math.degrees(gt_pose[2]), 1) if gt_pose else '',
                'gt_xy_error_m': round(gt_xy_err, 3) if gt_xy_err is not None else '',
                'gt_yaw_error_deg':
                    round(math.degrees(gt_yaw_err), 1) if gt_yaw_err is not None else '',
                'gt_success': int(gt_success) if gt_success is not None else '',
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

            time.sleep(SETTLE_PAUSE_S)

    node.get_logger().info(f'結果: {csv_path}')
    node.get_logger().info(
        '解析: ./gate1_analyze.py <csv> (成功率・誤差)。'
        'ドリフト蓄積の有無は前半/後半のgt_xy_error_m平均を比較すること')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
