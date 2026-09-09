import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

# 実機Go2のファームウェアが配信する`/utlidar/cloud`のheader.stampは
# **機体側の時計**で打たれており、開発PCの壁時計とは揃っていない。
# 2026-09-04の実測では機体が**1109.7秒(約18.5分)遅れて**いた(40秒間で安定)。
#
# 一方、同じチェーンに入る`state_to_odom_imu_node`のOdometry/Imuは
# (そのdocstringのとおり)ノード受信時刻=開発PCの壁時計でスタンプしている。
# この2つが混ざると、slam_toolboxがスキャン時刻(機体時計)でodom→base_linkのTFを
# 引こうとして必ず失敗し、**全スキャンが捨てられて地図が1枚も育たない**:
#
#   [slam_toolbox] Message Filter dropping message: frame 'base_link' at time
#   1788498317.243 for reason 'the timestamp on the message is earlier than
#   all the data in the transform cache'
#
# 機体側の時計を合わせられれば根本解決だが、実機の22/tcpは閉じておりログインできない。
# そこで受信時に開発PCの壁時計で打ち直して中継する。
#
# **なぜ「オフセットを測って引く」ではなく now() で打ち直すのか**: 機体の時計が
# NTP等で跳ねた場合に補正値が一瞬で無意味になるのと、そもそも機体時計の進み方が
# 壁時計と同じ保証が無いため。now()なら機体時計に一切依存しない。
# 代償は「転送遅延の分だけ実際の観測時刻より新しくなる」ことだが、
# 15Hz・歩行速度0.2m/s程度では誤差は数cm以下で、クソ雑map作成には十分。
#
# 使い方(既定のトピック名。launch側でremapしてもよい):
#   ros2 run go2_sport_bridge utlidar_cloud_restamp_node
#     cloud_in  <- /utlidar/cloud
#     cloud_out -> /utlidar/cloud_restamped

# ファームウェア側publisherのQoS(2026-09-04 `ros2 topic info -v`で実測):
#   Reliability: RELIABLE / History: KEEP_LAST(1) / Durability: VOLATILE
# BEST_EFFORTで購読するとRELIABLEなpublisherとは繋がる(要求が緩い側は互換)が、
# 明示的にRELIABLEを指定して意図を残す。
_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

# クロック差を報告する間隔(秒)。毎メッセージ出すと15Hzでログが流れるだけなので間引く。
_DEFAULT_REPORT_PERIOD = 10.0

# この値より差が小さければ「機体と開発PCの時計は揃っている」とみなし、
# 本ノードが不要である旨を1回だけ知らせる(将来ファームが時刻同期するようになった場合、
# このノードが挟まったままだと精度をわずかに損なうだけの存在になるため)。
_CLOCKS_AGREE_SEC = 0.2


class UtlidarCloudRestampNode(Node):
    """`/utlidar/cloud`のheader.stampを開発PCの壁時計で打ち直して中継する。

    frame_id・点群データそのものには一切手を触れない(stampだけを差し替える)。
    """

    def __init__(self):
        super().__init__('utlidar_cloud_restamp_node')

        self.declare_parameter('report_period', _DEFAULT_REPORT_PERIOD)
        self._report_period = float(
            self.get_parameter('report_period').get_parameter_value().double_value)

        self._pub = self.create_publisher(PointCloud2, 'cloud_out', _QOS)
        self._sub = self.create_subscription(PointCloud2, 'cloud_in', self._on_cloud, _QOS)

        self._count = 0
        self._last_report_ns = None
        self._agree_warned = False

        self.get_logger().info(
            'utlidar_cloud_restamp ready: cloud_in -> cloud_out '
            '(header.stampのみ受信時刻で打ち直す)。'
            f'クロック差は{self._report_period:.0f}秒ごとに報告する')

    def _on_cloud(self, msg):
        now = self.get_clock().now()

        firmware_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        offset_sec = (now.nanoseconds - firmware_ns) / 1e9

        msg.header.stamp = now.to_msg()
        self._pub.publish(msg)

        self._count += 1
        self._maybe_report(now.nanoseconds, offset_sec)

    def _maybe_report(self, now_ns, offset_sec):
        if self._last_report_ns is None:
            # 1通目は無条件に出す。ここが出ないまま黙っているときは
            # 購読側のQoS不一致か、そもそも実機が見えていない(GO2_NIC未指定)
            self.get_logger().info(
                f'機体クロックとの差: {offset_sec:+.3f}s '
                '(正=機体が遅れている)。この分だけstampを進めて中継する')
            self._last_report_ns = now_ns
            self._check_agreement(offset_sec)
            return

        if (now_ns - self._last_report_ns) / 1e9 >= self._report_period:
            self.get_logger().info(
                f'機体クロックとの差: {offset_sec:+.3f}s / 中継 {self._count} 通')
            self._last_report_ns = now_ns

    def _check_agreement(self, offset_sec):
        if self._agree_warned or abs(offset_sec) >= _CLOCKS_AGREE_SEC:
            return
        self._agree_warned = True
        self.get_logger().warn(
            f'機体と開発PCの時計の差が{abs(offset_sec):.3f}s'
            f'({_CLOCKS_AGREE_SEC}s未満)しかない。既に時刻が揃っているなら'
            'このノードは不要で、素の/utlidar/cloudを直接使う方がstampが正確になる')


def main(args=None):
    rclpy.init(args=args)
    node = UtlidarCloudRestampNode()
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
