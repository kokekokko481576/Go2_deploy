import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException


class StraightLinePlannerNode(Node):

    def __init__(self):
        super().__init__('straight_line_planner_node')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('path_resolution', 0.1)

        self._global_frame = self.get_parameter('global_frame').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._path_resolution = self.get_parameter('path_resolution').value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Nav2のplanner_serverが最終的に差し替わっても購読側が気づかないよう、
        # トピック名(goal_pose/plan)・型(PoseStamped/Path)をNav2に合わせている。
        path_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(Path, 'plan', path_qos)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)

        self.get_logger().info(
            'straight_line_planner ready: goal_pose -> plan '
            f'(frame={self._global_frame}, base={self._robot_base_frame}, '
            f'resolution={self._path_resolution}m)')

    def _lookup_current_pose(self, stamp):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {self._global_frame}->{self._robot_base_frame} not available: {ex}')
            return None

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = stamp
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _on_goal(self, goal):
        if goal.header.frame_id and goal.header.frame_id != self._global_frame:
            self.get_logger().warn(
                f'goal frame "{goal.header.frame_id}" != global_frame '
                f'"{self._global_frame}"; フレーム変換はせずそのまま座標を使う')

        stamp = self.get_clock().now().to_msg()
        start = self._lookup_current_pose(stamp)
        if start is None:
            return

        path = self._interpolate(start, goal, stamp)
        self._path_pub.publish(path)
        self.get_logger().info(
            f'published path: {len(path.poses)} points, '
            f'start=({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) '
            f'goal=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')

    def _interpolate(self, start, goal, stamp):
        sx, sy = start.pose.position.x, start.pose.position.y
        gx, gy = goal.pose.position.x, goal.pose.position.y
        dx, dy = gx - sx, gy - sy
        distance = math.hypot(dx, dy)
        # 進行方向を向かせる中間点の向き。始点=終点(距離ゼロ)の場合は現姿勢を維持する。
        heading = math.atan2(dy, dx) if distance > 1e-6 else self._yaw_of(start.pose.orientation)

        path = Path()
        path.header.frame_id = self._global_frame
        path.header.stamp = stamp

        num_segments = max(1, math.ceil(distance / self._path_resolution))
        for i in range(num_segments + 1):
            t = i / num_segments
            pose = PoseStamped()
            pose.header.frame_id = self._global_frame
            pose.header.stamp = stamp
            pose.pose.position.x = sx + dx * t
            pose.pose.position.y = sy + dy * t
            if i < num_segments:
                pose.pose.orientation = self._yaw_to_quaternion(heading)
            else:
                # 終端は目標作業姿勢(部材への正対)をそのまま使う
                pose.pose.orientation = goal.pose.orientation
            path.poses.append(pose)

        return path

    @staticmethod
    def _yaw_of(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = StraightLinePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
