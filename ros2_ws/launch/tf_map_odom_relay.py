#!/usr/bin/env python3
"""map->odom TF中継 (Go2_deploy #44 / RViz可視化用)。

フェーズBでは自作の自己位置推定(go2_localization)が map->odom->base_link を
`/go2_localization/tf` に、simのロボット(脚など)は `/robot1/tf` に配信する。
RVizは1つのtfトピックしか読めないため、このままでは「骨」と「map系(経路/AMCL)」を
同時に出せない。

このノードは `/go2_localization/tf` の中から **map->odom だけ** を取り出して
`/robot1/tf` へ再配信する。これで `/robot1/tf` が
  map -> odom -> base_link -> (脚)
の1本のツリーになり、RVizひとつで骨も経路もAMCL結果も表示できる。

odom->base_link はsim側(QuadrupedOdometry)が既に `/robot1/tf` に出しているので
重複を避けるため中継しない(map->odomのみ)。タイマ/TF参照を持たないため
use_sim_timeは不要(スタンプは元メッセージのまま転送)。
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class MapOdomRelay(Node):
    def __init__(self):
        super().__init__('tf_map_odom_relay')
        self._pub = self.create_publisher(TFMessage, '/robot1/tf', 10)
        self.create_subscription(TFMessage, '/go2_localization/tf', self._on_tf, 10)
        self.get_logger().info('tf_map_odom_relay: /go2_localization/tf(map->odom) -> /robot1/tf')

    def _on_tf(self, msg):
        relayed = [t for t in msg.transforms if t.header.frame_id == 'map']
        if relayed:
            self._pub.publish(TFMessage(transforms=relayed))


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
