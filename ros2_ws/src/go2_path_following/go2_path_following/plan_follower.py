"""straight_line_plannerが出すPathを、Nav2 controller_serverのFollowPathアクションの
ゴールとして送るだけの小さな橋渡しノード。

Nav2本来はbt_navigatorがFollowPathを呼ぶが、追M2(平地の経路追従・Issue #21)では
bt_navigatorを導入せず「生成済みPathを追従させてみる」最小構成にとどめるため、
このノードで代用する。新しいPathを受け取るたびに、実行中のゴールがあればキャンセルして
送り直すだけの単純な動作だった。

追M3(Issue #23)でリカバリを追加: FollowPathがprogress_checkerの停滞判定でABORTEDに
なった場合、その場回転→後退→再計画要求(最後のgoal_poseを再publishし、現在位置からの
新しいPathを生成させる)を行う。GATE1(#36)の50試行ベースラインで見えた「大きくズレた
失敗」(地図にない障害物に接触→無反応のままタイムアウト)への対策。
"""
from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node


class PlanFollower(Node):

    def __init__(self):
        super().__init__('plan_follower')
        self.declare_parameter('controller_id', 'FollowPath')
        self.controller_id = self.get_parameter('controller_id').value
        # Issue #22: `ros2 param set /plan_follower controller_id FollowPathMPPI`で
        # controller_serverの再起動なしにDWB/MPPIを切り替えられるようにする
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # リカバリ動作のパラメータ。速度はcontroller_server.yamlのFollowPath上限
        # (max_vel_theta=0.5, max_vel_x=0.18)を超えない値にする
        self.declare_parameter('recovery_spin_speed', 0.4)
        self.declare_parameter('recovery_spin_duration', 3.0)
        self.declare_parameter('recovery_backup_speed', -0.1)
        self.declare_parameter('recovery_backup_duration', 2.0)
        self.declare_parameter('recovery_cmd_rate', 10.0)
        # watchdog_timeout(cmd_vel_safety、既定0.5s)より十分短い周期で送り続ける必要がある
        self.declare_parameter('recovery_max_attempts', 3)

        self._spin_speed = self.get_parameter('recovery_spin_speed').value
        self._spin_duration = self.get_parameter('recovery_spin_duration').value
        self._backup_speed = self.get_parameter('recovery_backup_speed').value
        self._backup_duration = self.get_parameter('recovery_backup_duration').value
        self._max_attempts = self.get_parameter('recovery_max_attempts').value
        cmd_rate = self.get_parameter('recovery_cmd_rate').value

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._goal_handle = None
        # on_planでキャンセルした古いゴールの結果コールバックを無視するための世代カウンタ
        self._generation = 0

        self._last_goal_pose = None
        self._recovery_attempts = 0
        self._recovery_republish = False
        self._recovery_timer = None
        self._recovery_phase = None
        self._recovery_elapsed = 0.0
        self._cmd_period = 1.0 / cmd_rate

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self._goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10)

    def _on_set_parameters(self, params):
        for param in params:
            if param.name == 'controller_id':
                self.controller_id = param.value
                self.get_logger().info(f'controller_id を {self.controller_id} に切替')
        return SetParametersResult(successful=True)

    def _on_goal_pose(self, msg):
        # リカバリ完了後の再計画要求で使う「最後にユーザーが投入したゴール」を覚えておく
        self._last_goal_pose = msg
        if self._recovery_republish:
            # 自分がリカバリ完了時に再publishした分。同じゴールの継続なのでカウンタは
            # 維持する(ここで0に戻すとrecovery_max_attemptsが機能しなくなる)
            self._recovery_republish = False
        else:
            self._recovery_attempts = 0

    def on_plan(self, msg):
        if self._recovery_timer is not None:
            # 新しいPathが来た(再計画/別ゴール)ので進行中のリカバリは打ち切る
            self._cancel_recovery()

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'follow_path action server not available', throttle_duration_sec=5.0)
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        self._generation += 1
        generation = self._generation

        goal = FollowPath.Goal()
        goal.path = msg
        goal.controller_id = self.controller_id

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(future, generation))

    def _on_goal_response(self, future, generation):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath goal rejected')
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda future: self._on_result(future, generation))

    def _on_result(self, future, generation):
        if generation != self._generation:
            # 既に次のPathへ切り替わった後の古いゴールの結果。無視する
            return

        status = future.result().status
        self._goal_handle = None
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                'FollowPath ABORTED (進捗停滞または実行時エラー)。リカバリを開始します')
            self._start_recovery()
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._recovery_attempts = 0

    def _start_recovery(self):
        if self._last_goal_pose is None:
            self.get_logger().warn('リカバリ対象のgoal_poseが無いため中断')
            return
        self._recovery_attempts += 1
        if self._recovery_attempts > self._max_attempts:
            self.get_logger().error(
                f'リカバリ上限({self._max_attempts}回)に到達。'
                'このゴールへの追従を断念します(新しいgoal_poseを待ちます)')
            return

        self.get_logger().info(
            f'リカバリ試行 {self._recovery_attempts}/{self._max_attempts}: '
            'その場回転 → 後退 → 再計画要求')
        self._recovery_phase = 'spin'
        self._recovery_elapsed = 0.0
        self._recovery_timer = self.create_timer(self._cmd_period, self._on_recovery_tick)

    def _on_recovery_tick(self):
        self._recovery_elapsed += self._cmd_period
        twist = Twist()

        if self._recovery_phase == 'spin':
            if self._recovery_elapsed >= self._spin_duration:
                self._recovery_phase = 'backup'
                self._recovery_elapsed = 0.0
                twist.angular.z = 0.0
            else:
                twist.angular.z = self._spin_speed
        elif self._recovery_phase == 'backup':
            if self._recovery_elapsed >= self._backup_duration:
                self._finish_recovery()
                return
            twist.linear.x = self._backup_speed

        self._cmd_pub.publish(twist)

    def _finish_recovery(self):
        self._cancel_recovery()
        self._cmd_pub.publish(Twist())  # 明示的に停止
        self.get_logger().info('リカバリ完了。最後のgoal_poseを再publishして再計画を要求します')
        self._recovery_republish = True
        self._goal_pub.publish(self._last_goal_pose)

    def _cancel_recovery(self):
        if self._recovery_timer is not None:
            self._recovery_timer.cancel()
            self._recovery_timer = None
        self._recovery_phase = None


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
