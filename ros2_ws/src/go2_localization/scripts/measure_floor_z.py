#!/usr/bin/env python3
"""現在の床の高さ(base_link相対)を実測する。height_slice_vizのfloor_zが
2026-08-03のcafeワールド床高さ修正(commit 468e816)より前の値のままで
ズレていないか確認するため(Issue #56)。
"""
import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FloorZMeasure(Node):
    def __init__(self):
        super().__init__('floor_z_measure')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.done = False
        self.sub = self.create_subscription(
            PointCloud2, '/robot1/chin_lidar/scan/points', self.on_cloud,
            qos_profile_sensor_data)

    def on_cloud(self, msg):
        if self.done:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.2))
        except Exception as ex:
            self.get_logger().warn(f'tf failed: {ex}')
            return
        points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not points:
            return
        xyz = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        t = transform.transform
        rot = quaternion_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        origin = np.array([t.translation.x, t.translation.y, t.translation.z])
        xyz_bl = xyz @ rot.T + origin

        horiz = np.hypot(xyz_bl[:, 0], xyz_bl[:, 1])
        mask = (horiz > 0.5) & (horiz < 2.0)
        zs = xyz_bl[mask, 2]
        print(f'sensor_origin(base_link相対)={origin}')
        print(f'水平距離0.5〜2.0mの点数: {mask.sum()}')
        if mask.sum() > 0:
            print(f'z分布: min={zs.min():.3f} max={zs.max():.3f} '
                  f'median={np.median(zs):.3f} mean={zs.mean():.3f}')
            hist, edges = np.histogram(zs, bins=10)
            for h, e0, e1 in zip(hist, edges[:-1], edges[1:]):
                print(f'  [{e0:.3f},{e1:.3f}): {"#"*h} ({h})')
        self.done = True


def main():
    rclpy.init()
    node = FloorZMeasure()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
