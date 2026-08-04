"""`/goal_pose`(PoseStamped)を`bt_navigator`のNavigateToPoseアクションへ変換するだけの
薄い橋渡しノード。

BT導入(#23派生)により、経路生成・追従・リカバリの判断はすべてBT側(bt_navigator +
behavior_server)に移った。このノードは`send_goal.sh`/`gate1_measure.py`が使う
`/goal_pose`トピックのI/Fをそのまま残すためだけに存在する(旧`plan_follower`/
`plan_requester`が持っていたPath生成・追従・リカバリのロジックはBT XML
(`go2_bt_plugins/behavior_trees/navigate_with_collision_recovery.xml`)に移行済み)。
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridge(Node):

    def __init__(self):
        super().__init__('goal_pose_bridge')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10)
        self.get_logger().info(
            'goal_pose_bridge ready: goal_pose -> NavigateToPose(bt_navigator)')

    def _on_goal_pose(self, msg):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'navigate_to_pose action server not available', throttle_duration_sec=5.0)
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = msg

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda future: self._on_result(future, goal_handle))

    def _on_result(self, future, goal_handle):
        # 完了済みゴールのハンドルを残さない。次のgoal_poseで無駄なcancel_goal_asyncを
        # 投げないため(すでに新しいゴールに差し替わっていたら何もしない)
        if self._goal_handle is goal_handle:
            self._goal_handle = None
        status = future.result().status
        self.get_logger().info(f'NavigateToPose finished (status={status})')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
