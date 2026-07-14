"""straight_line_plannerが出すPathを、Nav2 controller_serverのFollowPathアクションの
ゴールとして送るだけの小さな橋渡しノード。

Nav2本来はbt_navigatorがFollowPathを呼ぶが、追M2(平地の経路追従・Issue #21)では
bt_navigatorを導入せず「生成済みPathを追従させてみる」最小構成にとどめるため、
このノードで代用する。新しいPathを受け取るたびに、実行中のゴールがあればキャンセルして
送り直すだけの単純な動作(リカバリ・再計画はM3以降で扱う)。
"""
import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node


class PlanFollower(Node):

    def __init__(self):
        super().__init__('plan_follower')
        self.declare_parameter('controller_id', 'FollowPath')
        self.controller_id = self.get_parameter('controller_id').value

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._goal_handle = None

        self.sub = self.create_subscription(Path, 'plan', self.on_plan, 10)

    def on_plan(self, msg):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'follow_path action server not available', throttle_duration_sec=5.0)
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal = FollowPath.Goal()
        goal.path = msg
        goal.controller_id = self.controller_id

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath goal rejected')
            return
        self._goal_handle = goal_handle


def main(args=None):
    rclpy.init(args=args)
    node = PlanFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
