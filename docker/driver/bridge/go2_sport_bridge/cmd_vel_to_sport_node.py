import json
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request

# unitree_ros2/example/src/include/common/ros2_sport_client.h の定義に合わせる
ROBOT_SPORT_API_ID_MOVE = 1008
ROBOT_SPORT_API_ID_STOP_MOVE = 1003


def _clamp_abs(value, limit):
    return max(-limit, min(limit, value))


class CmdVelToSportNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_sport_node')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        # 速度クランプ。cmd_vel_safety でもかかるが、ブリッジ単体で使う場合の保険。
        # **下限に注意**: Go2の歩容は0.15m/s程度からしか歩き出さず、それ未満は胴体が
        # 揺れるだけで進まない(2026-09-02実機実測)。上限をそこまで下げてはいけない。
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.2)
        self.declare_parameter('max_wz', 0.5)

        publish_rate = self.get_parameter('publish_rate').value
        self._timeout = self.get_parameter('watchdog_timeout').value
        self._max_vx = self.get_parameter('max_vx').value
        self._max_vy = self.get_parameter('max_vy').value
        self._max_wz = self.get_parameter('max_wz').value

        self._last_cmd = Twist()
        self._last_recv_time = None
        self._watchdog_triggered = False

        self._pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            'cmd_vel_to_sport ready: cmd_vel -> /api/sport/request '
            f'(Move, api_id={ROBOT_SPORT_API_ID_MOVE}) at {publish_rate}Hz, '
            f'watchdog={self._timeout}s, '
            f'上限 vx={self._max_vx} vy={self._max_vy} wz={self._max_wz}。'
            '事前にSport ClientでStandUp/BalanceStand'
            '(unitree_ros2_example go2_sport_client 4)を実行し、'
            '**機体を「通常モード」にしておくこと**(他モードではMoveを受けても歩かない)。'
        )

    def _next_id(self):
        """リクエストIDを毎回変える。

        **同じ id で同じ内容を送り続けると、機体が重複とみなして無視する。**
        2026-09-02の実機実測: 指令値が完全に一定になった瞬間から実速度が0.0000m/sになり、
        値が変わった瞬間だけ動いた。テレオペでキーを押しっぱなしにすると Twist が
        一定値になるため、id を固定したままではまさにこの条件に入り、機体が動かない。
        unitree_ros2 同梱の例は id を設定しないので、そのままでは連続的な速度制御ができない。
        """
        return self.get_clock().now().nanoseconds

    def _on_cmd_vel(self, msg):
        self._last_cmd = msg
        self._last_recv_time = self.get_clock().now()
        if self._watchdog_triggered:
            self._watchdog_triggered = False
            self.get_logger().info('cmd_vel の受信が再開しました')

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
                # StopMove は遷移時に1回だけ送る。
                # **連投してはいけない**: 20Hzで送り続けると機体の移動指令の受け付けを
                # 妨げ、リモコン操作とも競合する(2026-09-02実機実測)。
                self._publish_stop_move()
            # 途絶えている間はゼロ速度のMoveだけを送る。停止は保ちつつ、指令が
            # 再開したときにすぐ動ける状態を維持する。
            vx, vy, vyaw = 0.0, 0.0, 0.0
        else:
            vx = _clamp_abs(self._last_cmd.linear.x, self._max_vx)
            vy = _clamp_abs(self._last_cmd.linear.y, self._max_vy)
            vyaw = _clamp_abs(self._last_cmd.angular.z, self._max_wz)

        self._publish_move(vx, vy, vyaw)

    def _publish_move(self, vx, vy, vyaw):
        req = Request()
        req.header.identity.id = self._next_id()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE
        req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self._pub.publish(req)

    def _publish_stop_move(self):
        req = Request()
        req.header.identity.id = self._next_id()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STOP_MOVE
        self._pub.publish(req)

    def stop_move(self):
        """ゼロ速度と StopMove の両方を送る。どちらか一方では止まりきらない場合に備える。"""
        self._publish_move(0.0, 0.0, 0.0)
        self._publish_stop_move()


def main(args=None):
    # **rclpy既定のシグナルハンドラを使わない。**
    # 既定ではCtrl-Cでrclpyのコンテキストが先に落ち、spinが
    # ExternalShutdownExceptionを投げる。その時点でpublisherは無効になっているため、
    # 終了時に停止指令を送ろうとしても送れない(=最後の指令のまま機体が歩き続ける)。
    # ハンドラを無効化するとPython既定のSIGINT処理が働き、KeyboardInterruptが
    # 上がってくるだけでコンテキストは生きたままなので、停止指令を送ってから畳める。
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    # シグナルの扱いを明示的に決める。
    # - SIGINT: 非対話シェルのバックグラウンドジョブとして起動されるとSIG_IGNを
    #   継承してCtrl-Cが効かなくなるため、Python既定のハンドラへ戻す
    # - SIGTERM: `docker stop`等で飛んでくる。既定のままだと停止指令を出さずに
    #   即死するので、SIGINTと同じ経路(KeyboardInterrupt)に寄せる
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    node = CmdVelToSportNode()
    try:
        # **rclpy.spin()ではなく、短いタイムアウトのspin_onceを回す。**
        # 上記でrclpyのシグナルハンドラを外したので、Ctrl-CはPython既定の
        # KeyboardInterruptとして上がってくるが、Pythonのシグナル処理は
        # バイトコードの切れ目でしか走らない。spin()はC側で待ち続けるため
        # そのままではCtrl-Cが配送されず、ノードが終了しなくなる。
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # shutdown前に停止指令を出す。これを送らないと、最後の指令のまま歩き続ける恐れがある。
        # publish直後にプロセスが終わると送信が完了しないことがあるので少し待つ。
        if rclpy.ok():
            node.stop_move()
            time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
