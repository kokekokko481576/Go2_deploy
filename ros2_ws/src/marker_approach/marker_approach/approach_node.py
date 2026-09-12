"""マーカーに正対して所定の距離まで近づく接近制御（計画書の追M4 最終アプローチ相当）。

制御則そのものは `turn_drive_turn.py` にある（ROS非依存。`tools/sim_approach.py` で
実機なしに検証できる）。**制御則を直すときはあちらを読むこと。** このノードの仕事は

  1. マーカー姿勢(PoseStamped) を base_link 座標系の位置と法線に直す
  2. 安全（見失い・近づきすぎ・時間切れ・dry_run・enable）を見る
  3. 制御則を1周期まわして `cmd_vel_raw` に出す

出力は `cmd_vel_raw`。**必ず cmd_vel_safety を挟んでからドライバへ渡すこと**
（速度・加速度クランプとウォッチドッグはあちらの責務。ここでは重複して持たない）。

**vy（横移動）は一切出さない。** Go2は横移動がほとんど効かないどころか、
混ぜると前進そのものが止まる（vy=0.15の実効率1.3%、vy=-0.056で前進が停止）。
前身は vx/vy/wz を同時に出す全方向制御だったが、この事実により作り直した。

## 安全

- 既定では **enabled=False**。`~/enable` に True を送るまで速度を出さない
- マーカーを lost_timeout 秒見失ったら停止し、**enabled を False に落とす**（自動再開しない）
- min_distance より近づいたら停止
- max_runtime を超えたら停止
- dry_run=True（既定）のときは cmd_vel_raw を出さず、計算結果をログに出すだけ
- 現在の区間は `~/state` に出る（`ros2 topic echo` で見ておくと、次に機体が
  前進するのか旋回するのかが事前に分かる）
"""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from marker_approach.turn_drive_turn import Params, TurnDriveTurn


def yaw_of_quat(q):
    """geometry_msgs の quaternion からヨー角[rad]。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_of_array(q):
    """unitree_go の quaternion 配列 [w, x, y, z] からヨー角[rad]。"""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


class ApproachNode(Node):

    # ROSパラメータ名 -> (既定値, 制御則のパラメータ名)。角度は度で受けて rad に直す
    CONTROL_PARAMS = [
        ('standoff', 0.65, 'standoff'),
        ('pos_tolerance', 0.06, 'pos_tolerance'),
        ('ang_tolerance_deg', 5.2, 'ang_tolerance'),
        ('k_x', 0.6, 'k_x'),
        ('k_yaw', 1.0, 'k_yaw'),
        ('max_vx', 0.20, 'max_vx'),
        ('max_wz', 0.40, 'max_wz'),
        ('min_translation_speed', 0.15, 'min_translation_speed'),
        ('min_wz', 0.30, 'min_wz'),
        ('turn_tolerance_deg', 5.2, 'turn_tolerance'),
        ('redirect_tolerance_deg', 15.0, 'redirect_tolerance'),
        ('stop_lead_distance', 0.05, 'stop_lead_distance'),
        ('turn_lead_angle_deg', 4.0, 'turn_lead_angle'),
        ('fov_budget_deg', 25.0, 'fov_budget'),
        ('drive_bearing_limit_deg', 33.0, 'drive_bearing_limit'),
        ('settle_time', 0.7, 'settle_time'),
        ('max_cycles', 8, 'max_cycles'),
        ('min_progress', 0.03, 'min_progress'),
        ('min_ambiguity', 2.0, 'min_ambiguity'),
        ('normal_alpha', 0.3, 'normal_alpha'),
        # False にすると「マーカーの手前（視線上の standoff 点）」を狙う。
        # 届かない配置がなくなる代わりに、法線からのずれが残ったまま止まる
        ('use_normal', True, 'use_normal'),
        # 到達後の最終姿勢。'right'/'left' はマーカーを真横に入れる旋回を足す。
        # **この旋回にはヨー角の観測が要る**（下の yaw_source）
        ('final_heading', 'marker', 'final_heading'),
        ('side_turn_angle_deg', 90.0, 'side_turn_angle'),
    ]

    def __init__(self):
        super().__init__('marker_approach_node')

        for name, default, _ in self.CONTROL_PARAMS:
            self.declare_parameter(name, default)
        # base_link -> カメラ の取り付け（**実測して置き換えること**）
        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.0)
        self.declare_parameter('camera_pitch_deg', 0.0)   # 下向きを正
        # 安全
        self.declare_parameter('min_distance', 0.35)      # これより近づいたら停止[m]
        self.declare_parameter('dry_run', True)
        self.declare_parameter('lost_timeout', 0.5)
        self.declare_parameter('max_runtime', 120.0)
        self.declare_parameter('rate', 20.0)
        # ヨー角の入手先。真横へ旋回するときだけ要る（マーカーが視野から出るため）。
        #   none           : 使わない（final_heading='marker' のとき）
        #   imu            : sensor_msgs/Imu（Gazebo: /robot1/imu_plugin/out）**推奨**
        #   odometry       : nav_msgs/Odometry（Gazebo: /robot1/odometry/filtered）
        #   sportmodestate : unitree_go/SportModeState（実機: /sportmodestate）
        #
        # **脚オドメトリのヨーは使わないほうがよい。** Gazeboで90度旋回させたところ、
        # 真値では+94度回っているのにオドメトリは+86.5度としか報告せず、
        # 8%少なく見積もった（2026-09-08実測）。開ループの旋回はヨーの出所の
        # 誤差がそのまま最終姿勢に出る。IMUの積分のほうが滑りの影響を受けない。
        # **開始からの差分しか使わない**ので、絶対の基準・原点は問わない。
        self.declare_parameter('yaw_source', 'none')
        self.declare_parameter('yaw_topic', '/odom')
        # 推測航法の姿勢が1周期で跳ねたら、その観測を捨てる[rad]。
        # **Gazeboの `/robot1/odometry/filtered` はヨーが約180度飛ぶことがある**
        # （2026-09-08、真値・IMUが-86.4度のときodomだけ+93.3度）。これを使うと
        # 真横旋回が逆方向に回る。歩容の旋回は速くても0.5rad/s程度なので、
        # 1周期(0.05s)で30度も変わるのは観測の異常とみなしてよい。
        self.declare_parameter('odom_jump_limit_deg', 30.0)

        g = lambda n: self.get_parameter(n).value
        self.cam_xyz = (g('camera_x'), g('camera_y'), g('camera_z'))
        self.cam_pitch = math.radians(g('camera_pitch_deg'))
        self.min_distance = g('min_distance')
        self.dry_run = g('dry_run')
        self.lost_timeout = g('lost_timeout')
        self.max_runtime = g('max_runtime')
        rate = g('rate')

        kw = {}
        for name, _, ctl_name in self.CONTROL_PARAMS:
            v = g(name)
            kw[ctl_name] = math.radians(v) if name.endswith('_deg') else v
        # 視野の判定はカメラ位置で行う（base_linkではない）ので取り付けを渡す
        kw['camera_x'], kw['camera_y'] = self.cam_xyz[0], self.cam_xyz[1]
        params = Params(**kw)
        bad = params.validate()
        if bad:
            for b in bad:
                self.get_logger().error(f'パラメータが不整合です: {b}')
            raise SystemExit('パラメータを直してから起動してください')
        self.ctl = TurnDriveTurn(params)

        self.enabled = False
        self.started_at = None
        self.last_pose = None
        self.last_pose_time = None
        self.last_ambiguity = 0.0
        self.last_state = None
        self._abort_logged = False
        self._log_counter = 0

        self.last_odom = None   # (x, y, yaw)。真横旋回の推測航法に使う
        self.odom_jump_limit = math.radians(g('odom_jump_limit_deg'))
        self._odom_jump_warned = False
        self.yaw_source = g('yaw_source')
        self._setup_yaw(self.yaw_source, g('yaw_topic'), params)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        # 到達したときだけ True を1回出す。**打ち切り・失敗・見失いでは出さない。**
        # #66 の goal_pose_bridge.py が NavigateToPose の STATUS_SUCCEEDED でだけ出すのと
        # 同じ契約にしてあり、下流のアーム側ノードはトリガ源(Nav2 / マーカー接近)を
        # 区別せずに購読できる。相対名なので既定で /goal_reached に解決する。
        self.reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        self.create_subscription(PoseStamped, 'marker_pose', self.on_pose, 10)
        self.create_subscription(DiagnosticArray, 'marker_diagnostics', self.on_diag, 10)
        self.create_subscription(Bool, '~/enable', self.on_enable, 10)
        self.create_timer(1.0 / rate, self.tick)

        if self.cam_xyz == (0.0, 0.0, 0.0) and self.cam_pitch == 0.0:
            self.get_logger().warn(
                'base_link -> カメラ の取り付けが未設定です（すべて0）。'
                'カメラ座標系のまま制御するため、実機では正しくありません。実測値を渡してください。')
        self.get_logger().info(
            f'接近制御(turn-drive-turn) 準備完了 standoff={params.standoff}m  '
            f'{"ゴール=法線上（正対して止まる）" if params.use_normal else "ゴール=視線上（マーカーの手前。法線ずれは残る）"}  '
            f'{"[dry_run] cmd_vel_raw は出しません" if self.dry_run else "[実走行] cmd_vel_raw を出します"}  '
            f'enable=False（~/enable に true を送るまで動きません）')

    def _accept_odom(self, x, y, yaw):
        """推測航法の観測を受ける。**跳ねた観測は捨てる**（理由はパラメータの説明）。"""
        if self.last_odom is not None and self.last_odom[2] is not None:
            d = abs((yaw - self.last_odom[2] + math.pi) % (2 * math.pi) - math.pi)
            if d > self.odom_jump_limit:
                if not self._odom_jump_warned:
                    self._odom_jump_warned = True
                    self.get_logger().warn(
                        f'推測航法のヨーが1周期で {math.degrees(d):.0f}度 跳ねました。'
                        'この観測は捨てます（真横への旋回が逆向きに回る原因になります）')
                return
        self.last_odom = (x, y, yaw)

    def _setup_yaw(self, source, topic, params):
        """ヨー角の購読を用意する。**真横旋回を指定したのに供給が無い設定は起動時に弾く**
        （走ってから「回れません」で止まると、機体が中途半端な姿勢で残る）。"""
        if source == 'none':
            if params.final_heading != 'marker':
                raise SystemExit(
                    f'final_heading={params.final_heading!r} はヨー角が要ります。'
                    'yaw_source を odometry か sportmodestate にしてください'
                    '（90度回すとマーカーが視野から出るため、カメラでは閉じられません）')
            return
        if source == 'imu':
            # **IMUは向きしか出さない。** 真横旋回では機体が引きずられて動くので、
            # 位置が無いぶん誤差が残る（実測で方位10度ぶん）。位置の出るものを推奨
            from sensor_msgs.msg import Imu
            self.get_logger().warn(
                'yaw_source=imu は位置を出しません。真横への旋回では機体が'
                '7cm前・8cm横ほど引きずられ、その分だけ真横から外れます'
                '（2026-09-08 Gazebo実測）。位置の出る odometry / sportmodestate を推奨')
            self.create_subscription(
                Imu, topic,
                lambda m: self._accept_odom(None, None, yaw_of_quat(m.orientation)), 10)
        elif source == 'odometry':
            from nav_msgs.msg import Odometry
            self.create_subscription(
                Odometry, topic,
                lambda m: self._accept_odom(m.pose.pose.position.x, m.pose.pose.position.y,
                                            yaw_of_quat(m.pose.pose.orientation)), 10)
        elif source == 'sportmodestate':
            # unitree_go は実機環境にしか無いので、選ばれたときだけ import する
            from unitree_go.msg import SportModeState
            self.create_subscription(
                SportModeState, topic,
                lambda m: self._accept_odom(m.position[0], m.position[1],
                                            yaw_of_array(m.imu_state.quaternion)), 10)
        else:
            raise SystemExit(
                f'yaw_source={source!r} は不正。none/imu/odometry/sportmodestate')
        self.get_logger().info(f'ヨー角の購読: {source} <- {topic}')

    def _now(self):
        """時刻[s]。**ROS時計を使う**（`use_sim_time:=true` なら sim の時計）。

        `time.time()`（実時間）で測っていたときは、Gazeboが実時間の0.7倍で
        動いているぶん**区間の静止待ちがsim内では0.5秒しか無く**、旋回の惰性が
        終わる前に測り直していた。90度旋回の着地が真値で約9度ずれた原因がこれ。
        実機では use_sim_time=false なので実時間と一致し、挙動は変わらない。
        """
        return self.get_clock().now().nanoseconds / 1e9

    # ---- 入力 ----

    def on_enable(self, msg):
        if msg.data and not self.enabled:
            self.enabled = True
            self.started_at = self._now()
            self.ctl.start(self.started_at)
            self._abort_logged = False
            self.get_logger().info('有効化されました。接近を開始します。')
        elif not msg.data and self.enabled:
            self.disable('外部からの停止指示')

    def on_diag(self, msg):
        for st in msg.status:
            if st.name != 'marker_detector':
                continue
            for kv in st.values:
                if kv.key == 'ambiguity':
                    try:
                        self.last_ambiguity = float(kv.value)
                    except ValueError:
                        self.last_ambiguity = 0.0

    def on_pose(self, msg):
        self.last_pose = msg
        self.last_pose_time = self._now()

    # ---- 幾何 ----

    def marker_in_base(self):
        """マーカー位置と法線の方位を base_link 座標系で返す。"""
        p = self.last_pose.pose
        R = quat_to_matrix(p.orientation)
        # マーカー面の法線 = マーカー座標系のz軸（カメラ座標系での表現）
        n = [R[0][2], R[1][2], R[2][2]]
        m = [p.position.x, p.position.y, p.position.z]

        # カメラの取り付け（ピッチのみ考慮。ロール・ヨーはゼロ前提）
        c, s = math.cos(self.cam_pitch), math.sin(self.cam_pitch)
        # 下向きピッチ: カメラ座標のxを前・zを上として、y軸まわりに -pitch 回す
        def rot(v):
            return [c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]]
        m = rot(m); n = rot(n)
        m = [m[0] + self.cam_xyz[0], m[1] + self.cam_xyz[1], m[2] + self.cam_xyz[2]]

        # 法線はロボット側を向くように符号を揃える
        if n[0] * m[0] + n[1] * m[1] + n[2] * m[2] > 0:
            n = [-n[0], -n[1], -n[2]]
        return m, math.atan2(n[1], n[0])

    # ---- 制御 ----

    def disable(self, reason, error=False):
        self.enabled = False
        self.publish(0.0, 0.0)
        (self.get_logger().error if error else self.get_logger().warn)(f'停止しました: {reason}')

    def _alpha_note(self, cmd):
        """ログの alpha に付ける注記。**視線接近では alpha は観測しているが使っていない**
        （＝そのまま残る姿勢誤差）ので、未観測と区別が付くようにする。"""
        if not cmd.alpha_trusted:
            return '(未確定・視線で代用)'
        if not self.ctl.p.use_normal:
            return '(視線接近のため未使用＝このずれが残る)'
        return ''

    def publish(self, vx, wz):
        t = Twist()
        t.linear.x, t.angular.z = vx, wz      # vy は出さない（前進を殺すため）
        if not self.dry_run:
            self.cmd_pub.publish(t)
        return t

    def tick(self):
        if not self.enabled:
            return

        now = self._now()
        if self.started_at and now - self.started_at > self.max_runtime:
            self.disable(f'最大実行時間 {self.max_runtime}s を超過', error=True)
            return
        # **真横へ旋回する区間はマーカーが視野から出るのが正常**なので見失いで止めない。
        # 判定は制御則側の marker_optional() に一本化してある（前後の静止待ちも含む）。
        if not self.ctl.marker_optional():
            if self.last_pose_time is None or now - self.last_pose_time > self.lost_timeout:
                self.disable(f'マーカーを {self.lost_timeout}s 見失った', error=True)
                return

        m, gamma = self.marker_in_base()
        dist = math.hypot(m[0], m[1])
        # 見えていない区間では観測が古い。古い値で「近づきすぎ」を判定しない
        if not self.ctl.marker_optional() and dist < self.min_distance:
            self.disable(f'最小距離 {self.min_distance}m まで接近（実測 {dist:.3f}m）', error=True)
            return

        cmd = self.ctl.step(now, m[0], m[1], gamma, self.last_ambiguity, self.last_odom)
        self.publish(cmd.vx, cmd.wz)

        if cmd.state != self.last_state:
            self.last_state = cmd.state
            self.state_pub.publish(String(data=cmd.state))
            self.get_logger().info(f'== {cmd.state} == ({cmd.cycles + 1}周目)')

        # 打ち切りが決まった時点で理由を出す。**決定と停止の間に「正対へ旋回」が挟まる**ので、
        # 停止時にしか出さないと、画面上は理由なく正対を始めたように見える。
        if self.ctl.abort_reason is not None and not self._abort_logged:
            self._abort_logged = True
            self.get_logger().warn(
                f'打ち切りを決めました: {self.ctl.abort_reason}。'
                'マーカーへ向き直してから停止します')

        if cmd.done:
            if cmd.success:
                self.reached_pub.publish(Bool(data=True))
            self.disable(cmd.reason, error=not cmd.success)
            return

        self._log_counter += 1
        if self._log_counter % 10 == 1:
            self.get_logger().info(
                f'{cmd.state} | マーカー({m[0]:.3f},{m[1]:.3f}) 距離{cmd.dist:.3f}m '
                f'方位{math.degrees(cmd.bearing):+.1f}度'
                f'(カメラから{math.degrees(cmd.bearing_cam):+.1f}度) | '
                f'法線ずれalpha={math.degrees(cmd.alpha):+.1f}度'
                f'{self._alpha_note(cmd)} '
                f'曖昧性{self.last_ambiguity:.2f} | '
                f'ゴール誤差{cmd.pos_err * 1000:.0f}mm 方位{math.degrees(cmd.goal_bearing):+.1f}度 | '
                f'指令 vx={cmd.vx:+.3f} wz={cmd.wz:+.3f}'
                f'{" [dry_run]" if self.dry_run else ""}')


def main():
    rclpy.init()
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
