"""床除去後の点群を出すノード。元はRViz2のLaserScan表示が(原因不明で)描画できない
環境向けの可視化専用デバッグノードだったが、Issue #56でAMCL本体の入力にも
採用した(pointcloud_to_laserscanのcloud_inがこのノードの出力を見るようlaunchで
remap済み)。

単純な高さレンジでの輪切りだと、遠方の壁を拾うために低いmin_heightが必要になる一方、
その高さ帯には様々な距離で床も入り込んでしまい、床と壁を原理的に区別できなかった
(Issue #26)。そこで高さ範囲ではなく、各レイ(センサ原点からその点への直線)が
「何もない床」に当たるまでの理論距離を逆算し、実際の反射距離がそれより
floor_margin以上手前なら障害物(壁等)、理論距離付近〜以遠なら床とみなして除去する。
"""
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class HeightSliceViz(Node):

    def __init__(self):
        super().__init__('height_slice_viz')
        self.declare_parameter('target_frame', 'base_link')
        # 床の高さ(target_frame相対)。実測(cafe_worldの平地で静止スキャンした値)を仮値とする
        self.declare_parameter('floor_z', -0.3)
        # 理論上の床到達距離より、これ以上手前で反射していれば障害物とみなす
        self.declare_parameter('floor_margin', 0.1)
        # 参考程度の粗い上限(天井反射等の除外用)。床除去がメインなので緩めでよい
        self.declare_parameter('max_height', 0.3)

        # 本体(脚含む)の自己反射除外用の矩形(target_frame相対、Nav2 footprintと同じ値を既定に
        # 流用: go2_path_following/go2_path_planningのfootprint「[[0.37,0.18],[0.37,-0.18],
        # [-0.38,-0.18],[-0.38,0.18]]」)。range_minだけでは歩容で脚が前に振り出された時に
        # 除外しきれず、本体の一部が障害物として地図に焼き付く(歩き回るほど地図がえぐれる)。
        # 【重複注意】同じ矩形がconfig/pointcloud_to_laserscan.yaml(body_exclude_*)・
        # go2_path_following/go2_path_planningのfootprintにも手打ちされている。
        # footprintを更新する時は全部直すこと
        #
        # 【既知の制約】距離(range)ではなくbase_link相対のx/y位置だけで判定するため、
        # この矩形の真下にある本物の低い障害物(敷居・段差等、max_height以下のもの)も
        # 脚の自己反射と同様に除去されてしまい、SLAM地図上で見えなくなる。正しく区別する
        # には各脚の実際の関節位置をTFで追って除外範囲にする必要があるが、今回はそこまで
        # 実装しておらず、静的な矩形での簡易対応にとどめている
        self.declare_parameter('body_exclude_min_x', -0.38)
        self.declare_parameter('body_exclude_max_x', 0.37)
        self.declare_parameter('body_exclude_min_y', -0.18)
        self.declare_parameter('body_exclude_max_y', 0.18)

        # 2026-11-03: floor_zはtarget_frame(base_link)相対の固定値のため、歩行中に機体が
        # 前傾/傾斜すると(実測でピッチ±5度程度)理論床距離の計算がずれ、床を障害物として
        # 誤検出する(1m未満ビームが最大90%に達した)。これを補正するため、指定したIMUの
        # orientationからbase_link相対の重力方向を求め、floor_z/max_heightの判定を
        # 「base_linkのz軸」ではなく「重力方向への投影(=水平基準の高さ)」で行う。
        # 空文字(既定)なら無効化され、重力方向は[0,0,1]固定(=従来動作、姿勢水平前提)になる
        self.declare_parameter('imu_topic', '')

        # 2026-11-05: /utlidar/cloud_deskewed(frame_id=ファームウェア内部のodom、約1万点の蓄積点群)
        # をtf2でbase_linkへ変換すると、(1)自前EKFのodomはtwo_d_modeでz・roll・pitchが0に潰れて
        # おりファームウェアのodomと一致しない、(2)シングルスレッドでlookupが/tf受信を塞ぎ
        # バッファが約3秒遅れる、の2点で床除去が成立しなかった。指定するとtf2を使わず、
        # 同じodom系列のOdometry(/utlidar/robot_odom、3D姿勢入り)の時刻最近傍で変換する
        self.declare_parameter('pose_topic', '')
        self.declare_parameter('pose_max_age', 0.2)
        # pose_topic使用時の床判定。点群のframe(重力方向に揃い、2026-11-05実測で床は
        # z=-0.07〜0.0)のz値で直接判定する。機体の高さ・伏せ/起立・傾きに依存しないため、
        # floor_z・imu_topicによる補正はこのモードでは使わない
        self.declare_parameter('odom_floor_z', 0.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.floor_z = self.get_parameter('floor_z').value
        self.floor_margin = self.get_parameter('floor_margin').value
        self.max_height = self.get_parameter('max_height').value
        self.body_exclude_min_x = self.get_parameter('body_exclude_min_x').value
        self.body_exclude_max_x = self.get_parameter('body_exclude_max_x').value
        self.body_exclude_min_y = self.get_parameter('body_exclude_min_y').value
        self.body_exclude_max_y = self.get_parameter('body_exclude_max_y').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 水平基準の上方向をbase_link相対で表したベクトル。IMU未指定・未着時はこのまま
        # ([0,0,1]=target_frameのz軸=水平)で、floor_zの意味は従来通りtarget_frame相対の高さになる
        self.gravity_up_in_base = np.array([0.0, 0.0, 1.0])
        self._last_imu_update = 0.0
        imu_topic = self.get_parameter('imu_topic').value
        if imu_topic:
            self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)

        # 2026-11-05: 機体メイン基板(.161)が数十秒ずつ配信を止める現象があり、再開直後は
        # EKFが途切れ中の外挿から戻りきっておらずodom->base_linkがずれている。再開から
        # pose_resume_holdの間は点群を捨て、SLAMにずれた位置のスキャンを入れない
        self.declare_parameter('pose_resume_gap', 0.5)
        self.declare_parameter('pose_resume_hold', 0.5)
        self.pose_resume_gap = self.get_parameter('pose_resume_gap').value
        self.pose_resume_hold = self.get_parameter('pose_resume_hold').value
        self._last_pose_mono = None
        self._hold_until_mono = 0.0

        self._poses = deque(maxlen=50)
        self.pose_max_age = self.get_parameter('pose_max_age').value
        pose_topic = self.get_parameter('pose_topic').value
        if pose_topic:
            self.create_subscription(Odometry, pose_topic, self.on_pose, 10)

        self.sub = self.create_subscription(
            PointCloud2, 'cloud_in', self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'cloud_filtered', 10)

    def on_imu(self, msg):
        # 実機のIMUは300Hzで配信されており、毎回処理するとこのノードのCPU使用率が
        # 跳ね上がり、cloud_in側のコールバックが飢餓状態になって一切処理されなくなった
        # (2026-11-03に実機で確認)。姿勢は数十ms単位では殆ど変わらないので20Hzに間引く
        now = time.monotonic()
        if now - self._last_imu_update < 0.05:
            return
        self._last_imu_update = now

        o = msg.orientation
        # IMUのorientationはbase_link->参照座標系(odom等、水平基準)の回転。参照座標系の
        # 上方向[0,0,1]をbase_link相対で見るには逆回転(=転置、直交行列なので)を掛ける
        rot = quaternion_to_matrix(o.x, o.y, o.z, o.w)
        self.gravity_up_in_base = rot.T @ np.array([0.0, 0.0, 1.0])

    def on_pose(self, msg):
        now = time.monotonic()
        if self._last_pose_mono is not None:
            gap = now - self._last_pose_mono
            if gap > self.pose_resume_gap:
                self._hold_until_mono = now + self.pose_resume_hold
                self.get_logger().warn(
                    f'pose_topicが{gap:.1f}s途切れていた。再開後{self.pose_resume_hold}sは点群を捨てる')
        self._last_pose_mono = now
        self._poses.append(msg)

    def _pose_transform(self, msg):
        if time.monotonic() < self._hold_until_mono:
            raise RuntimeError('pose_topic途切れからの再開直後のため点群を捨てた')
        if not self._poses:
            raise RuntimeError('pose_topicがまだ1通も届いていない')
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        odom = min(self._poses,
                   key=lambda o: abs(Time.from_msg(o.header.stamp).nanoseconds - stamp))
        age = abs(Time.from_msg(odom.header.stamp).nanoseconds - stamp) / 1e9
        if age > self.pose_max_age:
            raise RuntimeError(f'時刻が最も近い姿勢でも{age:.3f}s離れている')
        if odom.header.frame_id != msg.header.frame_id:
            raise RuntimeError(
                f'点群のframe_id({msg.header.frame_id})と姿勢のframe_id'
                f'({odom.header.frame_id})が違う')
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        rot_odom_base = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        rot = rot_odom_base.T
        trans = -rot @ np.array([p.x, p.y, p.z])
        return rot, trans, p.z

    def _tf_transform(self, msg):
        transform = self.tf_buffer.lookup_transform(
            self.target_frame, msg.header.frame_id, msg.header.stamp,
            timeout=Duration(seconds=0.1))
        t = transform.transform
        rot = quaternion_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        trans = np.array([t.translation.x, t.translation.y, t.translation.z])
        return rot, trans, trans

    def on_cloud(self, msg):
        pose_mode = bool(self.get_parameter('pose_topic').value)
        try:
            if pose_mode:
                rot, trans, base_z = self._pose_transform(msg)
            else:
                rot, trans, sensor_origin = self._tf_transform(msg)
        except Exception as ex:
            self.get_logger().warn(
                f'transform {msg.header.frame_id} -> {self.target_frame} failed: {ex}',
                throttle_duration_sec=5.0)
            return

        # read_points_numpyは全フィールドが同じ型でないと使えない(cloud_baseはring/timeが別型)
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        # skip_nans=Trueはinf(検出なしを示す値)までは除いてくれないため別途フィルタ
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if pose_mode:
            # cloud_deskewedは固定長バッファで、約1万点のうち実点以外は(0,0,0)埋め
            xyz = xyz[np.any(xyz != 0.0, axis=1)]
        if xyz.shape[0] == 0:
            return

        xyz_out = xyz @ rot.T + trans

        if pose_mode:
            z = xyz[:, 2]
            floor_z = self.get_parameter('odom_floor_z').value
            mask = ((z > floor_z + self.floor_margin)
                    & (z <= base_z + self.max_height)
                    & ~self._self_body(xyz_out))
        else:
            mask = self._obstacle_mask(xyz_out, sensor_origin)
        filtered = xyz_out[mask]

        header = msg.header
        header.frame_id = self.target_frame
        cloud_out = point_cloud2.create_cloud_xyz32(header, filtered.tolist())
        self.pub.publish(cloud_out)

    def _obstacle_mask(self, points, sensor_origin):
        ray = points - sensor_origin
        actual_range = np.linalg.norm(ray, axis=1)

        # 高さ判定は target_frame の生のz軸ではなく、重力方向(水平基準の上方向、IMU未指定
        # 時は[0,0,1]でz軸と同じ)への投影で行う。機体が傾いても「本当の水平高さ」で
        # floor_z/max_heightを判定できる
        g = self.gravity_up_in_base
        ray_up = ray @ g
        origin_up = float(sensor_origin @ g)
        points_up = points @ g

        # このレイが床(水平基準でz=floor_z)に当たるまでの理論距離。
        # ray_up >= 0(水平以上/上向き)は床に当たらないレイなのでinf扱い(=常に障害物側)
        with np.errstate(divide='ignore', invalid='ignore'):
            floor_range = np.where(
                ray_up < 0,
                actual_range * (self.floor_z - origin_up) / ray_up,
                np.inf,
            )

        is_floor_hit = actual_range >= (floor_range - self.floor_margin)
        is_within_upper_bound = points_up <= self.max_height

        return (~is_floor_hit) & is_within_upper_bound & (~self._self_body(points))

    def _self_body(self, points):
        # 本体(脚含む)の自己反射: 距離に関係なく、base_link相対のx/y矩形内なら除外
        return (
            (points[:, 0] >= self.body_exclude_min_x)
            & (points[:, 0] <= self.body_exclude_max_x)
            & (points[:, 1] >= self.body_exclude_min_y)
            & (points[:, 1] <= self.body_exclude_max_y)
        )


def main(args=None):
    rclpy.init(args=args)
    node = HeightSliceViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
