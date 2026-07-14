"""RViz2のLaserScan表示が(原因不明で)描画できない環境向けの、高さスライス結果を
PointCloud2のまま可視化するためのデバッグ専用ノード。

pointcloud_to_laserscanと同じ「target_frameへ変換してから高さで輪切り」を行うが、
出力をLaserScanではなくPointCloud2のまま出すことで、動作確認済みのPointCloud2表示で
そのまま見えるようにする。AMCL本体のパイプライン(pointcloud_to_laserscan)には
一切手を加えず、可視化用の経路を並行して追加するだけの位置づけ。
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
        # pointcloud_to_laserscan.yamlのmin_height/max_heightと合わせること
        self.declare_parameter('min_height', -0.8)
        self.declare_parameter('max_height', 0.05)

        self.target_frame = self.get_parameter('target_frame').value
        self.min_height = self.get_parameter('min_height').value
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
        trans = np.array([t.translation.x, t.translation.y, t.translation.z])
        xyz_out = xyz @ rot.T + trans

        mask = (xyz_out[:, 2] >= self.min_height) & (xyz_out[:, 2] <= self.max_height)
        filtered = xyz_out[mask]

        header = msg.header
        header.frame_id = self.target_frame
        cloud_out = point_cloud2.create_cloud_xyz32(header, filtered.tolist())
        self.pub.publish(cloud_out)


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
