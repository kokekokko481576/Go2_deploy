import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState

# Unitree IMUState.quaternion は [w, x, y, z] 順
# (external/unitree_ros2/example/src/src/read_low_state.cpp のログ出力順で確認。
#  gitサブモジュールなので `git submodule update --init` していないと手元には無い)。
# ROS2 geometry_msgs/Quaternion は (x, y, z, w) 順なので並べ替えが要る。

# 以下は実測していない仮の対角共分散(REP-103の「未知」を表す-1にはしない=EKFに
# 使わせる)。全ゼロのままpublishすると`robot_localization`が「完全に確信度100%の
# 観測」と解釈し、他の入力より過剰に信用してしまう。実機到着後、実際のばらつきを
# 見て調整すること
_POSITION_VARIANCE = 0.01   # (m^2)
_YAW_VARIANCE = 0.05        # (rad^2)
_VELOCITY_VARIANCE = 0.01   # (m/s)^2 または (rad/s)^2
_ORIENTATION_VARIANCE = 0.05     # (rad^2)
_ANGULAR_VELOCITY_VARIANCE = 0.02   # (rad/s)^2
_LINEAR_ACCEL_VARIANCE = 0.1        # (m/s^2)^2


def _diag6(vx, vy, vz, vroll, vpitch, vyaw):
    cov = [0.0] * 36
    for i, v in enumerate((vx, vy, vz, vroll, vpitch, vyaw)):
        cov[i * 6 + i] = v
    return cov


def _diag3(v0, v1, v2):
    cov = [0.0] * 9
    cov[0] = v0
    cov[4] = v1
    cov[8] = v2
    return cov


class StateToOdomImuNode(Node):
    """実機Go2の`sportmodestate`(unitree_go/msg/SportModeState)を、
    go2_localizationのEKF/床除去チェーンがそのまま食えるnav_msgs/Odometry・
    sensor_msgs/Imuへ変換して配信する。sim側で upstream(gazebo_sim) が既に
    /robot1/odometry/filtered・/robot1/imu_plugin/out を出しているのと同じ役割を、
    実機では自前で用意する必要があるための橋渡し。

    未検証(実機到着後に確認すること):
    - SportModeState.velocityが機体座標系(child_frame_id=base_link相当)であるという前提
    - IMUの実搭載位置とbase_linkのズレ(frame_idはbase_link近似で代用)
    """

    def __init__(self):
        super().__init__('state_to_odom_imu_node')

        self._odom_pub = self.create_publisher(Odometry, '/go2_state_bridge/odom', 10)
        self._imu_pub = self.create_publisher(Imu, '/go2_state_bridge/imu', 10)
        self.create_subscription(SportModeState, 'sportmodestate', self._on_state, 10)

        self.get_logger().info(
            'state_to_odom_imu ready: sportmodestate -> '
            '/go2_state_bridge/odom (nav_msgs/Odometry) + '
            '/go2_state_bridge/imu (sensor_msgs/Imu)'
        )

    def _on_state(self, msg: SportModeState):
        stamp = self.get_clock().now().to_msg()
        qw, qx, qy, qz = msg.imu_state.quaternion

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        odom.pose.covariance = _diag6(
            _POSITION_VARIANCE, _POSITION_VARIANCE, _POSITION_VARIANCE,
            _YAW_VARIANCE, _YAW_VARIANCE, _YAW_VARIANCE)
        odom.twist.covariance = _diag6(
            _VELOCITY_VARIANCE, _VELOCITY_VARIANCE, _VELOCITY_VARIANCE,
            _VELOCITY_VARIANCE, _VELOCITY_VARIANCE, _VELOCITY_VARIANCE)
        self._odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.x = float(qx)
        imu.orientation.y = float(qy)
        imu.orientation.z = float(qz)
        imu.orientation.w = float(qw)
        imu.orientation_covariance = _diag3(
            _ORIENTATION_VARIANCE, _ORIENTATION_VARIANCE, _ORIENTATION_VARIANCE)
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.angular_velocity_covariance = _diag3(
            _ANGULAR_VELOCITY_VARIANCE, _ANGULAR_VELOCITY_VARIANCE, _ANGULAR_VELOCITY_VARIANCE)
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        imu.linear_acceleration_covariance = _diag3(
            _LINEAR_ACCEL_VARIANCE, _LINEAR_ACCEL_VARIANCE, _LINEAR_ACCEL_VARIANCE)
        self._imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = StateToOdomImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
