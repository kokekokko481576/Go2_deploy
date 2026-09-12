"""到達通知を受けて、D1アームを決め打ちの角度へ順に動かすノード。

## 何をするか

`goal_reached`(`std_msgs/Bool`) に True が来たら、`waypoints` で与えた関節角へ
`step_interval` 秒おきに1つずつ指令を出す。終わったら `arm_done` に True を出す。

トリガ源は問わない設計にしてある:

- Nav2 の `NavigateToPose` が成功したとき（#66、`goal_pose_bridge.py` が出す）
- マーカー接近が到達したとき（#74、`marker_approach/approach_node.py` が出す）

どちらも「成功したときだけ True を1回」という同じ契約なので、このノードは
どちらから呼ばれたかを知らなくてよい。

## なぜ「1関節ずつ」なのか（2026-09-12 にGazeboで実測）

**複数の関節を1つの指令でまとめて動かすと j1 が可動域上限(±2.36rad)まで走って
張り付く。** `[0.8, -0.6, 0.4, 0, 0.5, 0, 0, 0]` を1発で送ると j1 が +2.360 で
停止し、3回とも再現した。一方、

- 単軸だけ動かす指令（j1〜j6 を個別に ±0.4）は6軸とも指令どおり
- 2軸の同時指令（j1+j2 / j1+j3 / j1+j5）も指令どおり
- 同じ目標姿勢を **1関節ずつ積み上げて** 送ると +0.800/-0.600/+0.400/+0.500 に
  到達し、15秒後も保持していた

`docs/解説/D1アームのGazebo統合のしくみ.md` §8 にある「`d1_joint2` に 0.3 を指令すると
可動域上限まで振れて張り付く」（重力無効化で回避済み）と同じ系統の症状が、
多関節同時指令では残っているとみられる。`gz_ros2_control` の
`position_proportional_gain` がロボット全体で共通の1値であることが根にありそうだが、
**原因は未特定**（脚の歩行にも効く共通パラメータなので深追いは保留）。

したがって `waypoints` は「1行＝1つの中間姿勢」で書き、**隣り合う行の差分は
1関節だけにしておくこと**。既定値もそうしてある。実機側（#64）も
「30秒に1回の離散コマンド」方針なので、この作りはそのまま実機に持っていける。

## 使い方

    ros2 run d1_arm_demo arm_demo_node --ros-args \
      -r goal_reached:=/goal_reached \
      -r arm_command:=/robot1/d1_arm_controller/commands \
      -p step_interval:=4.0

実機では `step_interval` を 30.0 程度にする（`docs/計画/アーム動作.md` §4-3、
連続コマンドで数分後に無応答化するという他ラボの報告への対策）。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

# 関節の並び。`go2_description/config/ros_control.yaml` の d1_arm_controller の
# `joints:` と同じ順序でなければならない。
JOINT_NAMES = ('d1_joint1', 'd1_joint2', 'd1_joint3', 'd1_joint4',
               'd1_joint5', 'd1_joint6', 'd1_joint_l', 'd1_joint_r')
N_JOINTS = len(JOINT_NAMES)

# 既定のウェイポイント。**隣の行との差分は1関節だけ**（docstring 参照）。
# 中立(全0) から、ベースを振って肩・肘・手首を下へ向ける「足元を見る」姿勢まで。
# **sim での見た目合わせの暫定値。** 搭載位置TFも関節軸も実機未照合なので、
# 実機ではこの数値をそのまま使わないこと（`docs/計画/アーム動作.md` §5）。
DEFAULT_WAYPOINTS = [
    [0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # 中立（走行姿勢）
    [0.8,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # j1: ベースを振る
    [0.8, -0.6,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # j2: 肩を起こす
    [0.8, -0.6,  0.4, 0.0, 0.0, 0.0, 0.0, 0.0],   # j3: 肘を曲げる
    [0.8, -0.6,  0.4, 0.0, 0.5, 0.0, 0.0, 0.0],   # j5: 手首を下へ（撮影姿勢）
]


class ArmDemoNode(Node):

    def __init__(self):
        super().__init__('d1_arm_demo_node')

        self.declare_parameter('step_interval', 4.0)
        self.declare_parameter('return_to_neutral', True)
        self.declare_parameter('waypoints', [])

        self.step_interval = float(self.get_parameter('step_interval').value)
        self.return_to_neutral = bool(self.get_parameter('return_to_neutral').value)
        self.waypoints = self._load_waypoints()

        self.cmd_pub = self.create_publisher(Float64MultiArray, 'arm_command', 10)
        self.done_pub = self.create_publisher(Bool, 'arm_done', 10)
        self.create_subscription(Bool, 'goal_reached', self.on_goal_reached, 10)

        self.running = False
        self.index = 0
        self.timer = None

        self.get_logger().info(
            f'アームデモ 準備完了 ウェイポイント{len(self.waypoints)}点 '
            f'間隔{self.step_interval}s '
            f'{"（終了後に中立へ戻す）" if self.return_to_neutral else "（姿勢を保持したまま終わる）"} '
            'goal_reached に True が来るまで動きません')

    def _load_waypoints(self):
        """パラメータで渡されたウェイポイントを読む。平坦な配列を8個ずつに区切る。

        ROS2のパラメータは入れ子の配列を取れないので、`[0.0, 0.0, ... ]` を
        N_JOINTS の倍数の長さで渡してもらう形にしている。
        """
        raw = self.get_parameter('waypoints').value
        if not raw:
            return [list(w) for w in DEFAULT_WAYPOINTS]
        raw = [float(x) for x in raw]
        if len(raw) % N_JOINTS != 0:
            self.get_logger().error(
                f'waypoints の長さ {len(raw)} が {N_JOINTS} の倍数ではありません。'
                '既定のウェイポイントを使います')
            return [list(w) for w in DEFAULT_WAYPOINTS]
        return [raw[i:i + N_JOINTS] for i in range(0, len(raw), N_JOINTS)]

    def on_goal_reached(self, msg):
        if not msg.data:
            return
        if self.running:
            # **走行中の再通知は無視する。** 接近制御が何らかの理由で2回 True を
            # 出した場合に、途中から先頭へ巻き戻ると腕が予測できない動きをする
            self.get_logger().warn('動作中に到達通知が再度来ました。無視します')
            return
        self.get_logger().info('到達通知を受けました。アームを動かします')
        self.running = True
        self.index = 0
        # 最初の1点はすぐ出す。残りは step_interval おき
        self._send_next()
        self.timer = self.create_timer(self.step_interval, self._send_next)

    def _send_next(self):
        if self.index >= len(self.waypoints):
            self._finish()
            return
        wp = self.waypoints[self.index]
        self.cmd_pub.publish(Float64MultiArray(data=wp))
        changed = self._changed_joints(self.index)
        self.get_logger().info(
            f'[{self.index + 1}/{len(self.waypoints)}] '
            + ' '.join(f'{n.replace("d1_", "")}={v:+.3f}' for n, v in zip(JOINT_NAMES, wp))
            + (f'  （変化: {changed}）' if changed else '  （変化なし）'))
        self.index += 1

    def _changed_joints(self, i):
        """1つ前のウェイポイントから変わった関節の名前。**2つ以上あれば警告する。**"""
        if i == 0:
            return ''
        prev, cur = self.waypoints[i - 1], self.waypoints[i]
        names = [n.replace('d1_', '') for n, a, b in zip(JOINT_NAMES, prev, cur)
                 if abs(a - b) > 1e-9]
        if len(names) > 1:
            self.get_logger().warn(
                f'このウェイポイントで {len(names)} 関節が同時に動きます（{", ".join(names)}）。'
                'Gazeboでは多関節同時指令で j1 が可動域上限へ走る症状が出ています。'
                '1関節ずつに分けてください')
        return ', '.join(names)

    def _finish(self):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        if self.return_to_neutral:
            self.cmd_pub.publish(Float64MultiArray(data=[0.0] * N_JOINTS))
            self.get_logger().info('中立姿勢へ戻しました')
        self.running = False
        self.done_pub.publish(Bool(data=True))
        self.get_logger().info('アーム動作を完了しました')


def main():
    rclpy.init()
    node = ArmDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
