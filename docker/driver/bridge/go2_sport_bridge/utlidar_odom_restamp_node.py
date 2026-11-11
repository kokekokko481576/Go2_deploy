import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

# `utlidar_cloud_restamp_node.py`のOdometry版。`/utlidar/robot_odom`
# (LiDARベースのオドメトリと見られる、`/go2_state_bridge/odom`=sportmodestate由来の
# 関節+IMUオドメトリとは別系統)も、`/utlidar/cloud`と同様に**機体側の時計**で
# header.stampが打たれている(2026-11-03実測、現在時刻との差が`/utlidar/cloud`の
# restampで報告される値とほぼ一致)。そのままEKFに食わせると、PCの壁時計でスタンプされた
# IMU/他のodom入力と時刻が噛み合わずEKFが詰まる恐れがあるため、同じ手法(受信時にnow()で
# 打ち直す)で中継する。理由・設計方針はutlidar_cloud_restamp_node.pyのコメントと同じ。
#
# 2026-11-03: `/go2_state_bridge/odom`(関節エンコーダ+IMU)で作った地図がSLAM中に
# 大きく歪んでいた(IMU姿勢補正後も解消せず)ため、LiDARベースと見られるこちらの
# オドメトリ精度を試す目的で追加した。
#
# 使い方(既定のトピック名。launch側でremapしてもよい):
#   ros2 run go2_sport_bridge utlidar_odom_restamp_node
#     odom_in  <- /utlidar/robot_odom
#     odom_out -> /utlidar/robot_odom_restamped

# ファームウェア側publisherのQoS(2026-11-03 `ros2 topic info -v`で実測、cloudと同じ):
#   Reliability: RELIABLE / History: KEEP_LAST(1) / Durability: VOLATILE
_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

_DEFAULT_REPORT_PERIOD = 10.0
_CLOCKS_AGREE_SEC = 0.2
_OFFSET_JUMP_SEC = 1.0


class UtlidarOdomRestampNode(Node):
    """`/utlidar/robot_odom`のheader.stampを開発PCの壁時計で打ち直して中継する。

    frame_id・pose/twistの値そのものには一切手を触れない(stampだけを差し替える)。
    """

    def __init__(self):
        super().__init__('utlidar_odom_restamp_node')

        self.declare_parameter('report_period', _DEFAULT_REPORT_PERIOD)
        self._report_period = float(
            self.get_parameter('report_period').get_parameter_value().double_value)

        self._pub = self.create_publisher(Odometry, 'odom_out', _QOS)
        self._sub = self.create_subscription(Odometry, 'odom_in', self._on_odom, _QOS)

        self._count = 0
        self._last_report_ns = None
        self._last_report_offset = None
        self._agree_warned = False

        self.get_logger().info(
            'utlidar_odom_restamp ready: odom_in -> odom_out '
            '(header.stampのみ受信時刻で打ち直す)。'
            f'クロック差は{self._report_period:.0f}秒ごとに報告する')

    def _on_odom(self, msg):
        now = self.get_clock().now()

        firmware_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        offset_sec = (now.nanoseconds - firmware_ns) / 1e9

        msg.header.stamp = now.to_msg()
        self._pub.publish(msg)

        self._count += 1
        self._maybe_report(now.nanoseconds, offset_sec)

    def _maybe_report(self, now_ns, offset_sec):
        if self._last_report_ns is None:
            self.get_logger().info(
                f'機体クロックとの差: {offset_sec:+.3f}s '
                '(正=機体が遅れている)。この分だけstampを進めて中継する')
            self._last_report_ns = now_ns
            self._last_report_offset = offset_sec
            self._check_agreement(offset_sec)
            return

        if (now_ns - self._last_report_ns) / 1e9 >= self._report_period:
            self.get_logger().info(
                f'機体クロックとの差: {offset_sec:+.3f}s / 中継 {self._count} 通')
            jump = offset_sec - self._last_report_offset
            if abs(jump) >= _OFFSET_JUMP_SEC:
                self.get_logger().warn(
                    f'機体クロックとの差が前回の報告から {jump:+.3f}s 変わった。'
                    '機体の再起動・時刻同期など、機体側で時計が動いた可能性がある')
            self._last_report_ns = now_ns
            self._last_report_offset = offset_sec
            self._check_agreement(offset_sec)

    def _check_agreement(self, offset_sec):
        if self._agree_warned or abs(offset_sec) >= _CLOCKS_AGREE_SEC:
            return
        self._agree_warned = True
        self.get_logger().warn(
            f'機体と開発PCの時計の差が{abs(offset_sec):.3f}s'
            f'({_CLOCKS_AGREE_SEC}s未満)しかない。既に時刻が揃っているなら'
            'このノードは不要で、素の/utlidar/robot_odomを直接使う方がstampが正確になる')


def main(args=None):
    rclpy.init(args=args)
    node = UtlidarOdomRestampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
