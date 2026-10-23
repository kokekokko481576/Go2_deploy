#!/usr/bin/env python3
"""通し試験用の仮想世界（**sim 専用**。実機の代わりに ROS トピックを出し入れする）。2026-09-28

実機の接近制御ノード（marker_approach approach_node）と inspect_run.py を**そのまま**動かすために、
次の役を1つのノードで引き受ける:

  入力  /cmd_vel_raw              approach_node の指令 → sim_approach.Go2Sim（実測の遅れ・下限速度・揺れ）
  出力  /marker_detector_node/pose         検出器と同じ約束の PoseStamped（視野・距離・雑音・誤った解）
        /marker_detector_node/diagnostics  ambiguity
        /lowstate                          後脚股関節の温度（立っている間ゆっくり上がる）と電池

配置は sim_inspect_layout.py と同じ（マーカーが原点、法線 +x、撮影位置 (0.65,0) 向き -x、
部材は y=+0.342〜+0.642・x=0.457〜1.157）。終わったら撮影位置からのずれと部材とのすき間を
--out に JSON で書く（approach ノードの状態が「完了」になって2秒後）。
"""
import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String
from unitree_go.msg import LowState, SportModeState
from unitree_api.msg import Request

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_approach as S  # noqa: E402
from sim_approach import Go2Sim, observe, wrap  # noqa: E402
from sim_inspect_layout import clearance, STANDOFF  # noqa: E402

CAM_X, CAM_Z = 0.333, 0.035
WATCHDOG = 0.3     # ブリッジと同じ。指令が途切れたらゼロ


def quat_from_normal(g):
    """z軸がロボット座標で方位 g を向く姿勢（x=上向き）。"""
    # 列: x=(0,0,1), y=(sin g, -cos g, 0), z=(cos g, sin g, 0)
    m = [[0.0, math.sin(g), math.cos(g)],
         [0.0, -math.cos(g), math.sin(g)],
         [1.0, 0.0, 0.0]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


class World(Node):
    def __init__(self, a):
        super().__init__('sim_world')
        self.a = a
        yaw = wrap(math.atan2(-a.lat, -a.dist) + math.radians(a.yaw))
        self.sim = Go2Sim(a.dist, a.lat, yaw, seed=a.seed)
        self.rng = random.Random(a.seed + 1)
        self.t0 = time.time()
        self.t_last = 0.0
        self.last_cmd_t = None
        self.next_frame = 0.0
        self.min_clear = clearance(self.sim.x, self.sim.y, self.sim.yaw)
        self.done_at = None
        self.written = False
        self.hip0 = a.hip_start
        self.pose_pub = self.create_publisher(PoseStamped, '/marker_detector_node/pose', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/marker_detector_node/diagnostics', 10)
        self.low_pub = self.create_publisher(LowState, '/lowstate', 10)
        # 伏せ・起立（2026-09-29 追加）。/api/sport/request の StandDown(1005)/StandUp(1004) で体の高さを変える
        self.sport_pub = self.create_publisher(SportModeState, '/sportmodestate', 10)
        self.create_subscription(Request, '/api/sport/request', self.on_sport, 10)
        self.height, self.height_goal = 0.32, 0.32
        self.sport_log = []
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.create_subscription(String, '/marker_approach_node/state', self.on_state, 10)
        self.create_timer(0.01, self.step)
        self.create_timer(0.1, self.pub_low)
        self.create_timer(0.05, self.pub_sport)
        self.get_logger().info(f'仮想世界: 出発 マーカーから{a.dist:.2f}m 横{a.lat * 100:+.1f}cm '
                               f'向き{a.yaw:+.1f}度 seed={a.seed}')

    def t(self):
        return time.time() - self.t0

    def on_cmd(self, m):
        now = self.t()
        self.last_cmd_t = now
        self.sim.command(now, m.linear.x, m.angular.z)

    def on_sport(self, m):
        api = m.header.identity.api_id
        if api == 1005:
            self.height_goal = 0.10
        elif api == 1004:
            self.height_goal = 0.32
        self.sport_log.append((round(self.t(), 2), api))

    def pub_sport(self):
        # 0.1m/s で目標の高さへ（伏せ・起立に約2秒）
        d = self.height_goal - self.height
        self.height += max(-0.005, min(0.005, d))
        m = SportModeState()
        m.body_height = float(self.height)
        self.sport_pub.publish(m)

    def on_state(self, m):
        if m.data == '完了' and self.done_at is None:
            self.done_at = self.t()

    def step(self):
        now = self.t()
        if self.last_cmd_t is not None and now - self.last_cmd_t > WATCHDOG:
            self.sim.command(now, 0.0, 0.0)
            self.last_cmd_t = None
        while self.t_last < now:
            self.sim.advance(self.t_last, 0.01)
            self.t_last += 0.01
        self.min_clear = min(self.min_clear, clearance(self.sim.x, self.sim.y, self.sim.yaw))
        if now >= self.next_frame:
            self.next_frame = now + 1.0 / S.CAM_FPS
            moving = abs(self.sim.vx) > 0.02 or abs(self.sim.wz) > 0.05
            o = observe(self.sim, (0.0, 0.0), 0.0, self.rng, moving, now)
            if o is not None:
                mx, my, gamma, amb = o
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = 'camera'
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = mx - CAM_X, my, 0.0
                q = quat_from_normal(gamma)
                ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
                self.pose_pub.publish(ps)
                d = DiagnosticArray()
                st = DiagnosticStatus(name='marker_detector')
                st.values = [KeyValue(key='ambiguity', value=f'{amb:.3f}')]
                d.status = [st]
                self.diag_pub.publish(d)
        if self.done_at is not None and not self.written and now - self.done_at > 2.0:
            self.write_result()

    def pub_low(self):
        m = LowState()
        hip = self.hip0 + self.a.hip_rate * self.t() / 60.0
        for i in range(12):
            m.motor_state[i].temperature = int(hip if i in (6, 9) else 35)
        m.bms_state.soc = 60
        m.power_v = 30.0
        self.low_pub.publish(m)

    def write_result(self):
        s = self.sim
        r = dict(dist=self.a.dist, lat0=self.a.lat, yaw0=self.a.yaw, seed=self.a.seed,
                 along=-(s.x - STANDOFF), lateral=s.y, dyaw_deg=math.degrees(wrap(s.yaw - math.pi)),
                 min_clear=self.min_clear, t_done=self.done_at, sport_requests=self.sport_log,
                 height_now=round(self.height, 3))
        Path(self.a.out).write_text(json.dumps(r, ensure_ascii=False))
        self.written = True
        self.get_logger().info(f'結果: 横{r["lateral"] * 100:+.1f}cm 前後{r["along"] * 100:+.1f}cm '
                               f'向き{r["dyaw_deg"]:+.1f}度 部材とのすき間 最小{self.min_clear * 100:.1f}cm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=1.3)
    ap.add_argument('--lat', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--hip-start', type=float, default=40.0)
    ap.add_argument('--hip-rate', type=float, default=2.0, help='後脚股関節の上昇[℃/分]')
    ap.add_argument('--out', default='/tmp/sim_world_result.json')
    a = ap.parse_args()
    rclpy.init()
    n = World(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.done_at is not None:
            n.write_result()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
