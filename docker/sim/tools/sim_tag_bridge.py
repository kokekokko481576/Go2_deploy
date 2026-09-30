#!/usr/bin/env python3
"""Gazebo の apriltag_ros の出力を、実機の検出器と同じ `marker_pose` に変換する（**sim限定**）。

## なぜ要るか

実機側の検出ノード（GStreamerでGo2のH264を受け、較正値で歪みを補正して solvePnP）は
本リポジトリには未取り込み（#74 の実機側で追って入れる）。それが出すのと同じ
`PoseStamped` を、simでは apriltag_ros の結果から作る。Gazebo にはその経路が無いので、simイメージに
入っている `apriltag_ros` に検出させ、その結果を**同じ約束の同じ型**に直して流す。
こうすると接近制御（`marker_approach`）は実機と全く同じものが使える。

## 座標系（ここを間違えると全部ずれる）

`apriltag_ros` は **ROSの光学座標系**（x=右 / y=下 / z=前）で姿勢を出す。
実機の検出器は **ロボット座標系**（x=前 / y=左 / z=上、REP-103）に直して出している
（`marker_detector_node.cpp` の kOpticalToBody）。**同じ行列をここでも掛ける。**

なお Gazebo 側のカメラリンク `camera_face` は base_link と同じ向き（傾き無し）で
定義されているが、apriltag_ros はそれを光学座標系だと思って姿勢を出すので、
変換が要ることに変わりはない。

## simで再現していないもの（**結果の読み方に効く**）

- **姿勢の2解の曖昧性**。実機の精度の天井はこれで決まる（視線と法線のなす角が
  12度を切ると法線が読めなくなり、横ずれ standoff x sin12度 が残る）。
  apriltag_ros は第2解を出さないので、ここでは固定値を流している。
  **simで出た姿勢精度をそのまま実機の期待値にしてはいけない。**
- レンズ歪み（Gazeboのカメラは理想ピンホール）、検出の脱落、機体の揺れ

**コンテナの中(Jazzy)で動かすこと。** ホスト(Humble)から /tf を受けようとすると
`invalid data size, at serdata.cpp` で失敗する（ディストロ跨ぎのシリアライズ非互換）。
標準型なら跨いで通るという前提は、少なくともTFでは成り立たない。

使い方:
  docker exec go2-sim bash -c '. /opt/ros/jazzy/setup.bash && \
      python3 /sim_tools/sim_tag_bridge.py --ros-args -p tag_frame:=tag36h11:0'
  # 一式まとめて起動するなら docker/sim/tools/sim_up.sh
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

# 光学座標系(x右 y下 z前) -> ロボット座標系(x前 y左 z上)。
# marker_detector_node.cpp の kOpticalToBody と同じもの。
OPTICAL_TO_BODY = ((0.0, 0.0, 1.0),
                   (-1.0, 0.0, 0.0),
                   (0.0, -1.0, 0.0))


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matrix_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x, y, z = (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x, y, z = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x, y, z = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s
    return x, y, z, w


class SimTagBridge(Node):

    def __init__(self):
        super().__init__('sim_tag_bridge')
        # apriltag_ros がタグに付ける子フレーム名。既定は "tag36h11:<id>"。
        # 起動時に実際に流れている名前をログに出すので、違っていたら合わせること
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.declare_parameter('camera_frame', 'camera_face')
        # simでは第2解が得られないので固定値を流す。**実機の天井は再現していない**
        self.declare_parameter('ambiguity', 99.0)
        self.tag_frame = self.get_parameter('tag_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.ambiguity = self.get_parameter('ambiguity').value

        self.pose_pub = self.create_publisher(PoseStamped, 'marker_pose', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'marker_diagnostics', 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)
        self.create_timer(0.1, self.publish_diag)

        self.detections = 0
        self.seen_frames = set()
        self.last_report = 0.0
        self.get_logger().info(
            f'sim用の橋渡しを開始しました。{self.camera_frame} -> {self.tag_frame} の '
            'TFを marker_pose に変換します（光学座標系→ロボット座標系）')

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id != self.tag_frame:
                # 何が流れているかを一度だけ出す。フレーム名の取り違えは
                # 「静かに何も起きない」形で出るので、気づけるようにしておく
                if 'tag' in tr.child_frame_id and tr.child_frame_id not in self.seen_frames:
                    self.seen_frames.add(tr.child_frame_id)
                    self.get_logger().warn(
                        f'タグらしきフレーム {tr.child_frame_id!r} を見つけましたが、'
                        f'設定は {self.tag_frame!r} です。tag_frame を直してください')
                continue
            self.publish_pose(tr)

    def publish_pose(self, tr):
        t, q = tr.transform.translation, tr.transform.rotation
        # 位置: 光学 -> ロボット座標系
        v = (t.x, t.y, t.z)
        p = [sum(OPTICAL_TO_BODY[i][k] * v[k] for k in range(3)) for i in range(3)]
        # 姿勢: 同じ回転を左から掛ける（マーカー面の法線はz軸として下流が読む）
        r_body = matmul([list(row) for row in OPTICAL_TO_BODY],
                        quat_to_matrix(q.x, q.y, q.z, q.w))
        qx, qy, qz, qw = matrix_to_quat(r_body)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)
        self.detections += 1
        if self.detections == 1:
            self.get_logger().info(
                f'最初の検出: 前方{p[0]:.3f}m 左{p[1]:.3f}m 上{p[2]:.3f}m '
                f'（距離{math.hypot(p[0], p[1]):.3f}m）')

    def publish_diag(self):
        """**未検出でも出し続ける。** 実機側の検出器と同じ約束（下流が見失いを検知する）。"""
        d = DiagnosticArray()
        d.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = 'marker_detector'
        st.level = DiagnosticStatus.OK
        st.message = f'sim橋渡し（検出 {self.detections} 件）'
        st.values = [KeyValue(key='ambiguity', value=str(self.ambiguity)),
                     KeyValue(key='detections', value=str(self.detections))]
        d.status = [st]
        self.diag_pub.publish(d)


def main():
    rclpy.init()
    node = SimTagBridge()
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
