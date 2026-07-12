import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelSafetyNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_safety_node')

        self.declare_parameter('max_linear_x', 1.0)
        self.declare_parameter('max_linear_y', 0.5)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('watchdog_rate', 20.0)

        self._max_vx = self.get_parameter('max_linear_x').value
        self._max_vy = self.get_parameter('max_linear_y').value
        self._max_wz = self.get_parameter('max_angular_z').value
        self._max_lin_accel = self.get_parameter('max_linear_accel').value
        self._max_ang_accel = self.get_parameter('max_angular_accel').value
        self._timeout = self.get_parameter('watchdog_timeout').value
        watchdog_rate = self.get_parameter('watchdog_rate').value

        self._last_cmd = Twist()
        self._last_recv_time = None
        self._last_publish_time = self.get_clock().now()
        self._watchdog_triggered = False

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self._on_cmd_vel_raw, 10)
        self.create_timer(1.0 / watchdog_rate, self._on_watchdog_tick)

        self.get_logger().info(
            'cmd_vel_safety ready: cmd_vel_raw -> cmd_vel '
            f'(|vx|<={self._max_vx}, |vy|<={self._max_vy}, |wz|<={self._max_wz}, '
            f'watchdog={self._timeout}s)')

    def _on_cmd_vel_raw(self, msg):
        now = self.get_clock().now()
        self._last_recv_time = now
        self._watchdog_triggered = False
        safe = self._clamp(msg, now)
        self._publish(safe, now)

    def _on_watchdog_tick(self):
        now = self.get_clock().now()
        elapsed = None
        if self._last_recv_time is not None:
            elapsed = (now - self._last_recv_time).nanoseconds / 1e9

        if elapsed is None or elapsed > self._timeout:
            if not self._watchdog_triggered:
                self.get_logger().warn(
                    'cmd_vel watchdog triggered: no cmd_vel_raw for '
                    f'{elapsed:.2f}s (limit {self._timeout}s), stopping'
                    if elapsed is not None else
                    'cmd_vel watchdog triggered: no cmd_vel_raw received yet, stopping')
                self._watchdog_triggered = True
            # 緊急停止はレート制限を無視して即ゼロにする
            self._publish(Twist(), now)

    def _clamp(self, msg, now):
        vx = max(-self._max_vx, min(self._max_vx, msg.linear.x))
        vy = max(-self._max_vy, min(self._max_vy, msg.linear.y))
        wz = max(-self._max_wz, min(self._max_wz, msg.angular.z))

        dt = max((now - self._last_publish_time).nanoseconds / 1e9, 1e-3)
        vx = self._rate_limit(vx, self._last_cmd.linear.x, self._max_lin_accel, dt)
        vy = self._rate_limit(vy, self._last_cmd.linear.y, self._max_lin_accel, dt)
        wz = self._rate_limit(wz, self._last_cmd.angular.z, self._max_ang_accel, dt)

        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        return out

    @staticmethod
    def _rate_limit(target, previous, max_accel, dt):
        max_delta = max_accel * dt
        return max(previous - max_delta, min(previous + max_delta, target))

    def _publish(self, twist, now):
        self._pub.publish(twist)
        self._last_cmd = twist
        self._last_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
