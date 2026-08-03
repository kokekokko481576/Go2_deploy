import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request

# unitree_ros2/example/src/include/common/ros2_sport_client.h の定義に合わせる
ROBOT_SPORT_API_ID_MOVE = 1008


class CmdVelToSportNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_sport_node')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('watchdog_timeout', 0.5)

        publish_rate = self.get_parameter('publish_rate').value
        self._timeout = self.get_parameter('watchdog_timeout').value

        self._last_cmd = Twist()
        self._last_recv_time = None
        self._watchdog_triggered = False

        self._pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            'cmd_vel_to_sport ready: cmd_vel -> /api/sport/request '
            f'(Move, api_id={ROBOT_SPORT_API_ID_MOVE}) at {publish_rate}Hz, '
            f'watchdog={self._timeout}s. 事前にSport ClientでStandUp/BalanceStand'
            '(unitree_ros2_example go2_sport_client 4)を実行しておくこと。'
        )

    def _on_cmd_vel(self, msg):
        self._last_cmd = msg
        self._last_recv_time = self.get_clock().now()
        self._watchdog_triggered = False

    def _on_timer(self):
        now = self.get_clock().now()
        elapsed = None
        if self._last_recv_time is not None:
            elapsed = (now - self._last_recv_time).nanoseconds / 1e9

        if elapsed is None or elapsed > self._timeout:
            if not self._watchdog_triggered:
                self.get_logger().warn(
                    'cmd_vel watchdog triggered: no cmd_vel for '
                    f'{elapsed:.2f}s (limit {self._timeout}s), Move(0,0,0)'
                    if elapsed is not None else
                    'cmd_vel watchdog triggered: no cmd_vel received yet, Move(0,0,0)')
                self._watchdog_triggered = True
            vx, vy, vyaw = 0.0, 0.0, 0.0
        else:
            vx = self._last_cmd.linear.x
            vy = self._last_cmd.linear.y
            vyaw = self._last_cmd.angular.z

        self._publish_move(vx, vy, vyaw)

    def _publish_move(self, vx, vy, vyaw):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE
        req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self._pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSportNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
