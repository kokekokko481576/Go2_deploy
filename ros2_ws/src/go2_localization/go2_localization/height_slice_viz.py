"""RViz2のLaserScan表示が(原因不明で)描画できない環境向けの、床除去後の点群を
PointCloud2のまま可視化するためのデバッグ専用ノード。

単純な高さレンジでの輪切りだと、遠方の壁を拾うために低いmin_heightが必要になる一方、
その高さ帯には様々な距離で床も入り込んでしまい、床と壁を原理的に区別できなかった
(Issue #26)。そこで高さ範囲ではなく、各レイ(センサ原点からその点への直線)が
「何もない床」に当たるまでの理論距離を逆算し、実際の反射距離がそれより
floor_margin以上手前なら障害物(壁等)、理論距離付近〜以遠なら床とみなして除去する。

AMCL本体のパイプライン(pointcloud_to_laserscan)には一切手を加えず、可視化用の
経路を並行して追加するだけの位置づけ。
"""
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


class HeightSliceViz(Node):

    def __init__(self):
        super().__init__('height_slice_viz')
        self.declare_parameter('target_frame', 'base_link')
        # 床の高さ(target_frame相対)。実測(cafe_worldの平地で静止スキャンした値)を仮値とする
        self.declare_parameter('floor_z', -0.3)
        # 理論上の床到達距離より、これ以上手前で反射していれば障害物とみなす
        self.declare_parameter('floor_margin', 0.1)
        # 参考程度の粗い上限(天井反射等の除外用)。床除去がメインなので緩めでよい
        self.declare_parameter('max_height', 0.3)

        self.target_frame = self.get_parameter('target_frame').value
        self.floor_z = self.get_parameter('floor_z').value
        self.floor_margin = self.get_parameter('floor_margin').value
        self.max_height = self.get_parameter('max_height').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, 'cloud_in', self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'cloud_filtered', 10)

    def on_cloud(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.1))
        except Exception as ex:
            self.get_logger().warn(
                f'transform {msg.header.frame_id} -> {self.target_frame} failed: {ex}',
                throttle_duration_sec=5.0)
            return

        points = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not points:
            return
        xyz = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)
        # skip_nans=Trueはinf(検出なしを示す値)までは除いてくれないため別途フィルタ
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0] == 0:
            return

        t = transform.transform
        rot = quaternion_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        sensor_origin = np.array([t.translation.x, t.translation.y, t.translation.z])
        xyz_out = xyz @ rot.T + sensor_origin

        filtered = xyz_out[self._obstacle_mask(xyz_out, sensor_origin)]

        header = msg.header
        header.frame_id = self.target_frame
        cloud_out = point_cloud2.create_cloud_xyz32(header, filtered.tolist())
        self.pub.publish(cloud_out)

    def _obstacle_mask(self, points, sensor_origin):
        ray = points - sensor_origin
        actual_range = np.linalg.norm(ray, axis=1)

        # このレイが床(target_frame相対z=floor_z)に当たるまでの理論距離。
        # ray_z >= 0(水平以上/上向き)は床に当たらないレイなのでinf扱い(=常に障害物側)
        ray_z = ray[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            floor_range = np.where(
                ray_z < 0,
                actual_range * (self.floor_z - sensor_origin[2]) / ray_z,
                np.inf,
            )

        is_floor_hit = actual_range >= (floor_range - self.floor_margin)
        is_within_upper_bound = points[:, 2] <= self.max_height
        return (~is_floor_hit) & is_within_upper_bound


def main(args=None):
    rclpy.init(args=args)
    node = HeightSliceViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
