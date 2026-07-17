"""goal_poseをNav2 planner_serverのComputePathToPoseアクションに変換し、
結果のPathをplanトピックへ流すだけの小さな橋渡しノード。

Nav2本来はbt_navigatorがこのアクションを呼ぶが、生M2(Issue #16/#17)では
bt_navigatorを導入せず「planner_serverで計画したPathを既存の下流
(plan_follower→controller_server)にそのまま食わせる」最小構成にとどめる。
straight_line_planner(生M1)と購読/配信のI/F(goal_pose/plan、QoS含む)を
揃えてあるので、どちらか一方を起動するだけで下流は差し替えに気づかない。
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class PlanRequester(Node):

    def __init__(self):
        super().__init__('plan_requester')
        self.declare_parameter('planner_id', 'GridBased')
        self.planner_id = self.get_parameter('planner_id').value

        self._action_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')

        # straight_line_plannerと同じQoS(後着のsubscriberにも最新Pathが届くlatched相当)
        path_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(Path, 'plan', path_qos)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)

        self.get_logger().info(
            f'plan_requester ready: goal_pose -> ComputePathToPose({self.planner_id}) -> plan')

    def _on_goal(self, msg):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'compute_path_to_pose action server not available',
                throttle_duration_sec=5.0)
            return

        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.planner_id = self.planner_id
        goal.use_start = False  # 現在位置(TF)を始点に使う

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('ComputePathToPose goal rejected')
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if not result.path.poses:
            self.get_logger().warn(
                '計画結果が空(ゴールが障害物内/コストマップ外、またはTF未取得の可能性)')
            return
        self._path_pub.publish(result.path)
        self.get_logger().info(
            f'published path: {len(result.path.poses)} points, '
            f'planning_time={result.planning_time.sec + result.planning_time.nanosec / 1e9:.3f}s')


def main(args=None):
    rclpy.init(args=args)
    node = PlanRequester()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
