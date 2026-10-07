"""到達通知を受けて、D1アームを決め打ちの角度へ順に動かすノード。

## 何をするか

`goal_reached`(`std_msgs/Bool`) に True が来たら、`waypoints` で与えた関節角へ
`step_interval` 秒おきに1つずつ指令を出す。終わったら `arm_done` に True を出す。

**`arm_done` は「指令を送り終えた」通知であって「アームが着いた」ではない。**
このノードはフィードバックを購読していない(実機のフィードバックは `rt/` なしの
トピックで ROS2 からは購読できない。d1_arm_bridge の README 参照)。到達を確かめたい
下流は、`d1_sdk/run.sh get_arm_joint_angle` で実角度を読むこと。

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

したがって **送る指令はどれも、1つ前の指令から1関節しか変わらないようにしてある**
(`plan.py`)。ウェイポイントの間・開始姿勢への移動・中立復帰のすべてが対象で、
中立へは来た道を逆にたどって戻る。2関節以上変わるウェイポイントを渡すと、
起動時に警告したうえで1関節ずつに分けて送る。

起動時のアームは中立姿勢にあるものとみなす(走行中は中立に固定する前提。README参照)。
実機側（#64）も「30秒に1回の離散コマンド」方針なので、この作りはそのまま実機に持っていける。

## 使い方

    ros2 run d1_arm_demo arm_demo_node --ros-args \
      -r goal_reached:=/goal_reached \
      -r arm_command:=/robot1/d1_arm_controller/commands \
      -p step_interval:=4.0

`step_interval` の既定は実機向けの 30.0（`docs/計画/アーム動作.md` §4-3、
連続コマンドで数分後に無応答化するという他ラボの報告への対策）。**実機では
d1_arm_bridge の `min_command_interval` 以上にすること。** sim では上のように短くしてよい。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64MultiArray

from d1_arm_demo.plan import build_plan, changed_joints, multi_joint_rows

# 関節の並び。`go2_description/config/ros_control.yaml` の d1_arm_controller の
# `joints:` と同じ順序でなければならない。
JOINT_NAMES = ('d1_joint1', 'd1_joint2', 'd1_joint3', 'd1_joint4',
               'd1_joint5', 'd1_joint6', 'd1_joint_l', 'd1_joint_r')
N_JOINTS = len(JOINT_NAMES)
NEUTRAL = [0.0] * N_JOINTS
DEFAULT_STEP_INTERVAL = 30.0

# 既定のウェイポイント。**動かすのは j1 と j2 の2軸だけ**（#64 の方針。残り4軸と
# グリッパーは中立に固定する）。**隣の行との差分は1関節だけ**（docstring 参照）。
#
# 2軸で足りる理由（2026-09-12 に base_link -> d1_link6 のTFで実測）:
#   先端のピッチは **j2 + j3 + j5 の単純な和**（j2=0.8→45.8度、j3=0.8→45.8度、
#   どちらも 1.57→89.95度で一致）。つまり「カメラをどちらへ向けるか」だけなら
#   ピッチ軸は1つで足りる。j1 が方位、j2 が俯角を決める。
#
#   中立      : 先端 x=0.232 z=0.502 ピッチ0度（真正面・水平）
#   j2=+1.2   : 先端 x=0.342 z=0.041 ピッチ68.8度（床まで0.34m）  ← これを使う
#   j2=+1.57  : 先端 x=0.261 z=-0.091 ピッチ90.0度（床まで0.21m。真下向き）
#   j1=+1.0   : 先端が y=-0.330 へ（**正で右**。接近制御の final_heading=right と同じ向き）
#
#   j2=1.57 の真下向きは床まで0.21mしかなく、デプスカメラの最小測距（D435 で約0.3m）を
#   割る。**俯角を1つ浅くして 1.2 にし、床まで0.34mを確保する**方を既定にした。
#
# 2軸の制約: ピッチ軸が1つなので「先端の高さ・前後位置」と「カメラの向き」が連動する。
# 両方を独立に決めたければ j3 を足して3軸にする。角度決め打ちで1通り決まればよい
# 今回の用途では2軸で足りる。
#
# **sim での幾何合わせの暫定値。** 搭載位置TFも関節軸も実機未照合なので、
# 実機ではこの数値をそのまま使わないこと（`docs/計画/アーム動作.md` §5）。
DEFAULT_WAYPOINTS = [
    [0.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # 中立（走行姿勢）
    [1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # j1: ベースを右真横へ
    [1.57, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # j2: 俯角68.8度（撮影姿勢）
]


class ArmDemoNode(Node):

    def __init__(self):
        super().__init__('d1_arm_demo_node')

        self.declare_parameter('step_interval', DEFAULT_STEP_INTERVAL)
        self.declare_parameter('return_to_neutral', True)
        # 型を明示する。既定値を空リストにすると BYTE_ARRAY と推論され、
        # double 配列を渡すと起動時に InvalidParameterTypeException で落ちる
        self.declare_parameter('waypoints', Parameter.Type.DOUBLE_ARRAY)

        self.step_interval = float(self.get_parameter('step_interval').value)
        if self.step_interval <= 0.0:
            # 負だと最初の到達通知の瞬間に create_timer が例外で落ち、0だと全力で撃ち続ける
            self.get_logger().error(
                f'step_interval={self.step_interval} は正でなければなりません。'
                f'既定の {DEFAULT_STEP_INTERVAL}s を使います')
            self.step_interval = DEFAULT_STEP_INTERVAL
        self.return_to_neutral = bool(self.get_parameter('return_to_neutral').value)
        self.waypoints = self._load_waypoints()
        for i in multi_joint_rows(self.waypoints):
            names = ', '.join(JOINT_NAMES[j].replace('d1_', '')
                              for j in changed_joints(self.waypoints[i - 1], self.waypoints[i]))
            self.get_logger().warn(
                f'ウェイポイント{i + 1}行目は前の行から複数の関節が変わります（{names}）。'
                'Gazeboでは多関節同時指令で j1 が可動域上限へ走るため、1関節ずつに分けて送ります。'
                '動かす順序を決めたい場合は1関節ずつの行に書き直してください')

        self.cmd_pub = self.create_publisher(Float64MultiArray, 'arm_command', 10)
        self.done_pub = self.create_publisher(Bool, 'arm_done', 10)
        self.create_subscription(Bool, 'goal_reached', self.on_goal_reached, 10)

        self.running = False
        self.plan = []
        self.index = 0
        self.timer = None
        # 最後に送った姿勢。起動時は中立とみなす
        self.current = list(NEUTRAL)

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
        # 型だけ宣言した未指定のパラメータは、Humble では get_parameter が
        # ParameterUninitializedException を投げるので get_parameter_or で読む
        raw = self.get_parameter_or('waypoints', None).value
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
        self.plan = build_plan(self.current, self.waypoints, self.return_to_neutral, NEUTRAL)
        self.index = 0
        self.get_logger().info(f'{len(self.plan)}回に分けて送ります（{self.step_interval}sおき）')
        # 最初の1点はすぐ出す。残りは step_interval おき
        self._send_next()
        self.timer = self.create_timer(self.step_interval, self._send_next)

    def _send_next(self):
        if self.index >= len(self.plan):
            self._finish()
            return
        wp = self.plan[self.index]
        changed = ', '.join(JOINT_NAMES[j].replace('d1_', '')
                            for j in changed_joints(self.current, wp))
        self.cmd_pub.publish(Float64MultiArray(data=wp))
        self.current = list(wp)
        self.get_logger().info(
            f'[{self.index + 1}/{len(self.plan)}] '
            + ' '.join(f'{n.replace("d1_", "")}={v:+.3f}' for n, v in zip(JOINT_NAMES, wp))
            + f'  （変化: {changed}）')
        self.index += 1

    def _finish(self):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        self.running = False
        # 送信完了の通知(到達の確認ではない。docstring 参照)
        self.done_pub.publish(Bool(data=True))
        self.get_logger().info('アームへの指令を送り終えました（到達は確認していません）')


def main():
    rclpy.init()
    node = ArmDemoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGTERM では rclpy が先にコンテキストを畳むので、二重に shutdown しない
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
