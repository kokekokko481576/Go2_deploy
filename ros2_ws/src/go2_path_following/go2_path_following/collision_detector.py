"""障害物接触を検知して`collision_detected`(Bool)を発行するノード(Issue #23派生)。

GATE1(#36)の検証で、壁に接触した際に脚キネマティクスのオドメトリ(`/robot1/odom`)が
滑りを検知できずに前進を積算し続け、AMCLの補正(map→odom)も追いつかず「実際には
未到達なのに座標上だけゴールに到達したように見える」ことが判明した(#14参照)。
`progress_checker`(10秒無進捗でABORTED)より速く反応して滑る時間そのものを減らすため、
2種類の検知を行う:

1. LiDAR近距離: 正面近距離に反射が居続けているのに前進を指令している → 接触中とみなす
2. IMU/オドメトリ不一致: IMUの加速度を自前で短時間積分した推定変位が、
   `/robot1/odom`(脚キネマティクス)が主張する変位よりずっと小さい → 空転(滑り)とみなす
   (既存のEKF出力同士を比べても、EKFがIMUで踏ん張れておらず検知できなかったため、
   IMU生データを直接使う)

検知したら`collision_detected`(std_msgs/Bool)をpublishするだけで、リカバリの実行は
`plan_follower`(既存の#23リカバリ: その場回転→後退→再計画要求)に任せる。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool


def _rotate_by_quaternion(qx, qy, qz, qw, vx, vy, vz):
    """ボディ座標のベクトル(vx,vy,vz)をクォータニオン(qx,qy,qz,qw)でワールド座標へ回転"""
    # v' = q * v * q^-1 を展開した式
    ux, uy, uz = qx, qy, qz
    dot_uv = ux * vx + uy * vy + uz * vz
    dot_uu = ux * ux + uy * uy + uz * uz
    cross_x = uy * vz - uz * vy
    cross_y = uz * vx - ux * vz
    cross_z = ux * vy - uy * vx
    rx = 2.0 * dot_uv * ux + (qw * qw - dot_uu) * vx + 2.0 * qw * cross_x
    ry = 2.0 * dot_uv * uy + (qw * qw - dot_uu) * vy + 2.0 * qw * cross_y
    rz = 2.0 * dot_uv * uz + (qw * qw - dot_uu) * vz + 2.0 * qw * cross_z
    return rx, ry, rz


class CollisionDetector(Node):

    def __init__(self):
        super().__init__('collision_detector')

        self.declare_parameter('lidar_scan_topic', '/go2_localization/chin_lidar_scan')
        self.declare_parameter('lidar_range_threshold', 0.3)
        self.declare_parameter('lidar_front_half_angle_deg', 30.0)
        self.declare_parameter('lidar_min_duration', 1.0)
        self.declare_parameter('cmd_forward_threshold', 0.02)
        self.declare_parameter('imu_window_duration', 1.0)
        self.declare_parameter('imu_odom_min_disp', 0.05)
        self.declare_parameter('imu_max_disp', 0.02)
        self.declare_parameter('trigger_hold_duration', 1.0)
        self.declare_parameter('check_rate', 10.0)
        self.declare_parameter('gravity', 9.8)

        self._range_threshold = self.get_parameter('lidar_range_threshold').value
        self._front_half_angle = math.radians(
            self.get_parameter('lidar_front_half_angle_deg').value)
        self._lidar_min_duration = self.get_parameter('lidar_min_duration').value
        self._cmd_forward_threshold = self.get_parameter('cmd_forward_threshold').value
        self._imu_window_duration = self.get_parameter('imu_window_duration').value
        self._imu_odom_min_disp = self.get_parameter('imu_odom_min_disp').value
        self._imu_max_disp = self.get_parameter('imu_max_disp').value
        self._trigger_hold_duration = self.get_parameter('trigger_hold_duration').value
        self._gravity = self.get_parameter('gravity').value
        check_period = 1.0 / self.get_parameter('check_rate').value

        self._last_cmd = Twist()
        self._last_scan = None
        self._lidar_block_ticks = 0
        self._lidar_block_ticks_needed = max(1, round(self._lidar_min_duration / check_period))

        self._last_odom_xy = None
        self._window_start_odom_xy = None
        self._window_start_time = None
        self._imu_vel_xy = [0.0, 0.0]
        self._imu_disp_xy = [0.0, 0.0]
        self._imu_last_time = None

        self._trigger_until = None

        lidar_topic = self.get_parameter('lidar_scan_topic').value
        # LaserScan/Imuはセンサ系のBEST_EFFORT配信(pointcloud_to_laserscan・gazebo imuプラグイン
        # とも既定でsensor_data QoS)なので、こちら側もsensor_data QoSで揃える必要がある
        # (RELIABLEのままだとQoS不整合で1件も受信できない)
        self.create_subscription(LaserScan, lidar_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_raw', self._on_cmd, 10)
        self.create_subscription(Odometry, '/robot1/odom', self._on_odom, 10)
        self.create_subscription(
            Imu, '/robot1/imu_plugin/out', self._on_imu, qos_profile_sensor_data)
        self._pub = self.create_publisher(Bool, 'collision_detected', 10)

        self.create_timer(check_period, self._on_check_tick)
        self.get_logger().info(
            f'collision_detector ready: lidar<{self._range_threshold}m '
            f'{self._lidar_min_duration}s + imu/odom不一致 で collision_detected を発行')

    def _on_scan(self, msg):
        self._last_scan = msg

    def _on_cmd(self, msg):
        self._last_cmd = msg

    def _on_odom(self, msg):
        self._last_odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_imu(self, msg):
        now = self.get_clock().now()
        if self._imu_last_time is None:
            self._imu_last_time = now
            return
        dt = (now - self._imu_last_time).nanoseconds / 1e9
        self._imu_last_time = now
        if dt <= 0.0 or dt > 0.5:
            return  # 起動直後・タイムジャンプ時の異常値を無視

        q = msg.orientation
        wx, wy, _ = _rotate_by_quaternion(
            q.x, q.y, q.z, q.w,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # 重力(ワールドz)を除いた水平方向の純加速度
        ax, ay = wx, wy
        self._imu_vel_xy[0] += ax * dt
        self._imu_vel_xy[1] += ay * dt
        self._imu_disp_xy[0] += self._imu_vel_xy[0] * dt
        self._imu_disp_xy[1] += self._imu_vel_xy[1] * dt

    def _front_min_range(self, scan):
        min_range = math.inf
        angle = scan.angle_min
        for r in scan.ranges:
            if -self._front_half_angle <= angle <= self._front_half_angle:
                if scan.range_min <= r <= scan.range_max and r < min_range:
                    min_range = r
            angle += scan.angle_increment
        return min_range

    def _check_lidar(self):
        forward_commanded = abs(self._last_cmd.linear.x) > self._cmd_forward_threshold
        blocked = False
        if self._last_scan is not None and forward_commanded:
            blocked = self._front_min_range(self._last_scan) < self._range_threshold

        if blocked:
            self._lidar_block_ticks += 1
        else:
            self._lidar_block_ticks = 0
        return self._lidar_block_ticks >= self._lidar_block_ticks_needed

    def _check_imu_odom_mismatch(self):
        now = self.get_clock().now()
        if self._window_start_time is None:
            self._window_start_time = now
            self._window_start_odom_xy = self._last_odom_xy
            return False

        elapsed = (now - self._window_start_time).nanoseconds / 1e9
        if elapsed < self._imu_window_duration:
            return False

        mismatch = False
        if self._window_start_odom_xy is not None and self._last_odom_xy is not None:
            odom_disp = math.hypot(
                self._last_odom_xy[0] - self._window_start_odom_xy[0],
                self._last_odom_xy[1] - self._window_start_odom_xy[1])
            imu_disp = math.hypot(self._imu_disp_xy[0], self._imu_disp_xy[1])
            forward_commanded = abs(self._last_cmd.linear.x) > self._cmd_forward_threshold
            if (forward_commanded and odom_disp > self._imu_odom_min_disp
                    and imu_disp < self._imu_max_disp):
                mismatch = True
                self.get_logger().warn(
                    f'IMU/オドメトリ不一致: odom変位={odom_disp:.3f}m '
                    f'IMU推定変位={imu_disp:.3f}m (空転の疑い)')

        # 次の窓へリセット
        self._window_start_time = now
        self._window_start_odom_xy = self._last_odom_xy
        self._imu_vel_xy = [0.0, 0.0]
        self._imu_disp_xy = [0.0, 0.0]
        return mismatch

    def _on_check_tick(self):
        now = self.get_clock().now()
        lidar_trigger = self._check_lidar()
        imu_trigger = self._check_imu_odom_mismatch()

        if lidar_trigger or imu_trigger:
            hold_ns = int(self._trigger_hold_duration * 1e9)
            self._trigger_until = now + rclpy.duration.Duration(nanoseconds=hold_ns)
            if lidar_trigger:
                self.get_logger().warn('LiDAR近距離接触を検知')

        detected = self._trigger_until is not None and now < self._trigger_until
        self._pub.publish(Bool(data=detected))


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
