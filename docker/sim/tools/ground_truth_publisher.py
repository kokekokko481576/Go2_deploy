#!/usr/bin/env python3
"""Gazebo物理演算上の真値位置を/gz_ground_truth/poseへpublishする、sim限定の検証ツール(Issue #43)。

【重要】これはシミュレーション限定のデバッグ/計測ツールであり、実機には存在しない。
自己位置推定(AMCL/EKF、/go2_localization/*)とは完全に別のトピック名前空間
(/gz_ground_truth/*)を使い、自己位置推定システムには一切書き込まない
(/go2_localization/tf 等は変更しない)。実機移行時はこのファイルと
docker/sim/tools/ 以下を削除するだけで、他のコードに影響なく取り除ける
(どのノードもこのトピックをsubscribeしない設計にすること)。

このスクリプトは`ros2_ws`パッケージの一部ではない。simコンテナ(Gazebo Jazzy側)に
`docker cp`で送り込んで実行する単体スクリプト(start_ground_truth.sh参照)。
`ros2_ws`側(devコンテナ、ROS2 Humble)のパッケージは一切参照しない。

実装: `gz topic -e -t /world/<world>/pose/info`(gz.msgs.Pose_V)をsubprocessで購読し、
対象エンティティ(既定robot1_my_bot)のブロックだけをテキストパースして
geometry_msgs/msg/PoseStampedとして配信する。ros_gz_bridgeのPose_V→TFMessage変換は
このJazzy環境ではエンティティ名(name)が失われる既知の制限があり使えなかったため、
`gz topic -e`の出力を直接パースする方式にした。
"""
import argparse
import re
import subprocess
import sys
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

_FLOAT = r'([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)'
_NAME_RE = re.compile(r'^\s*name:\s*"([^"]*)"')
_FIELD_RE = re.compile(r'^\s*(x|y|z|w):\s*' + _FLOAT)


class GroundTruthPublisher(Node):

    def __init__(self, world, entity_name, gz_cmd):
        super().__init__('gz_ground_truth_publisher')
        self._entity_name = entity_name
        self._world = world
        self._gz_cmd = gz_cmd
        self._pub = self.create_publisher(PoseStamped, '/gz_ground_truth/pose', 10)
        self._proc = None
        self._stop = False
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'gz_ground_truth_publisher ready: world={world} entity={entity_name} '
            '-> /gz_ground_truth/pose (frame_id=map、sim限定・実機には存在しない)')

    def destroy_node(self):
        self._stop = True
        if self._proc is not None:
            self._proc.terminate()
        super().destroy_node()

    def _run_loop(self):
        topic = f'/world/{self._world}/pose/info'
        while not self._stop:
            try:
                self._proc = subprocess.Popen(
                    [self._gz_cmd, 'topic', '-e', '-t', topic],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                self._parse_stream(self._proc.stdout)
            except FileNotFoundError:
                self.get_logger().error(f'"{self._gz_cmd}" コマンドが見つかりません')
                return
            except Exception as exc:  # noqa: BLE001 - 再接続のため広めに捕捉
                self.get_logger().warn(f'gz topic購読が切れました: {exc}。3秒後に再試行')
            if not self._stop:
                threading.Event().wait(3.0)

    def _parse_stream(self, stream):
        depth = 0
        target_depth = None
        name = None
        pos = {}
        orient = {}
        section = None  # 'position' | 'orientation' | None

        for line in stream:
            if self._stop:
                return
            stripped = line.strip()

            if stripped.endswith('{'):
                depth += 1
                if stripped.startswith('position'):
                    section = 'position'
                elif stripped.startswith('orientation'):
                    section = 'orientation'
                elif target_depth is None and re.match(r'^pose\b', stripped):
                    # 新しいエンティティのpose {}ブロック開始
                    target_depth = depth
                    name, pos, orient, section = None, {}, {}, None
                continue

            if stripped == '}':
                if section is not None and depth == (target_depth or 0) + 1:
                    section = None
                depth -= 1
                if target_depth is not None and depth < target_depth:
                    # 対象ブロックが閉じた
                    if name == self._entity_name and 'x' in pos and 'w' in orient:
                        self._publish(pos, orient)
                    target_depth = None
                continue

            m = _NAME_RE.match(line)
            if m is not None:
                name = m.group(1)
                continue

            m = _FIELD_RE.match(line)
            if m is not None and section is not None:
                field, value = m.group(1), float(m.group(2))
                (pos if section == 'position' else orient)[field] = value

    def _publish(self, pos, orient):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = pos.get('x', 0.0)
        msg.pose.position.y = pos.get('y', 0.0)
        msg.pose.position.z = pos.get('z', 0.0)
        msg.pose.orientation.x = orient.get('x', 0.0)
        msg.pose.orientation.y = orient.get('y', 0.0)
        msg.pose.orientation.z = orient.get('z', 0.0)
        msg.pose.orientation.w = orient.get('w', 1.0)
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='default')
    parser.add_argument('--entity-name', default='robot1_my_bot')
    parser.add_argument('--gz-cmd', default='gz')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = GroundTruthPublisher(args.world, args.entity_name, args.gz_cmd)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
