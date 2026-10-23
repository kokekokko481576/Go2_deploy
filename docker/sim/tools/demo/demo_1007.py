#!/usr/bin/env python3
"""Gazebo（go2-sim コンテナ内）で、10/7 の3つの変更を見せる流れを動かす（動画用）。

    ① 収納を真後ろへ   止まったまま、旧収納（angle0=68.7、肘が左後ろ）を見せてから新収納（0、腕が機体の軸に沿う）へ回す。
                       収納し終えてから歩き出し、接近の間は収納の指令を送り続けてアームを固定する
    ② 速い展開         伏せたあと、inspect_poses.json の deploy_fast（3段）で撮影姿勢へ。かかった秒数を出す
    ③ 撮影位置の補正   アーム先端の深度カメラで試し撮り → aim_correct.py で隅肉の根元を見つける → 直す → 撮る

9/29 の demo_flow.py と同じ作法（伏せ = STAND＋cmd_vel.linear.z、アームは d1_arm_controller へ rad で少しずつ）。
③ は inspect_run.py と同じ aim_correct.correct_loop を使い、撮る・動かす部分だけ Gazebo につなぐ。
録画ノード（recorder_1007.py）向けに、段階を /demo_status、検出の重ね画像を /tmp/demo_aim/ に出す。

   python3 demo_1007.py [撮影姿勢で待つ秒数]
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from quadropted_msgs.msg import RobotModeCommand

sys.path.insert(0, '/marker_detection/tools')
import aim_correct  # noqa: E402

POSES = '/marker_detection/tools/inspect_poses.json'
OLD_STOW = [68.7, -89.3, 91.1, 0, 0, 0]
ARM_JOINTS = ['d1_joint1', 'd1_joint2', 'd1_joint3', 'd1_joint4', 'd1_joint5', 'd1_joint6', 'd1_joint_l', 'd1_joint_r']
ARM_SPEED = math.radians(40.0)
SHOT_DIR = '/tmp/demo_shots'
AIM_DIR = '/tmp/demo_aim'


class Demo(Node):
    def __init__(self):
        super().__init__('demo_1007')
        self.status_pub = self.create_publisher(String, '/demo_status', 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, '/robot1/d1_arm_controller/commands', 10)
        self.mode_pub = self.create_publisher(RobotModeCommand, '/robot1/robot_mode', 10)
        self.vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/marker_approach_node/enable', 10)
        self.joints = None
        self.arm_img = None
        self.arm_depth = None
        self.cam_k = None
        self.approach_result = None
        self.create_subscription(JointState, '/robot1/joint_states', self.on_js, 10)
        self.create_subscription(Image, '/robot1/arm_cam/image', lambda m: setattr(self, 'arm_img', m), 2)
        self.create_subscription(Image, '/robot1/arm_cam/depth_image', lambda m: setattr(self, 'arm_depth', m), 2)
        self.create_subscription(CameraInfo, '/robot1/arm_cam/color/camera_info',
                                 lambda m: setattr(self, 'cam_k', list(m.k)), 2)
        self.create_subscription(Log, '/rosout', self.on_log, 50)
        self.arm_cmd = None
        self.cur_deg = None      # 最後に送った SDK の angle0〜5[度]

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM_JOINTS):
            self.joints = [d[j] for j in ARM_JOINTS]

    def on_log(self, m):
        if m.name.endswith('marker_approach_node') and '停止しました' in m.msg:
            self.approach_result = ('到達しました' in m.msg, m.msg)

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def status(self, s):
        self.get_logger().info(s)
        for _ in range(3):
            self.status_pub.publish(String(data=s))
            self.spin_for(0.05)

    # ---- アーム ----
    def arm_to(self, sdk_deg, speed=ARM_SPEED):
        """SDK の angle0〜5[度] へ、関節ごとに一定速度で（最も遠い関節に合わせて同時に着く）。"""
        target = [math.radians(v) for v in sdk_deg[:6]] + [0.0, 0.0]
        if self.arm_cmd is None:
            while self.joints is None:
                self.spin_for(0.1)
            self.arm_cmd = list(self.joints)
        start = list(self.arm_cmd)
        dist = max(abs(t - s) for t, s in zip(target, start))
        dur = max(0.3, dist / speed)
        n = max(1, int(dur * 30))
        for k in range(1, n + 1):
            a = k / n
            a = a * a * (3 - 2 * a)
            cmd = [s + (t - s) * a for s, t in zip(start, target)]
            self.arm_pub.publish(Float64MultiArray(data=cmd))
            self.arm_cmd = cmd
            self.spin_for(1.0 / 30)
        self.spin_for(0.3)
        self.cur_deg = [float(v) for v in sdk_deg[:6]]

    # ---- 姿勢 ----
    def mode(self, name):
        for _ in range(3):
            self.mode_pub.publish(RobotModeCommand(mode=name, robot_id=1))
            self.spin_for(0.1)

    def body_z(self, direction, sec):
        t = Twist()
        t.linear.z = float(direction)
        t0 = time.time()
        while time.time() - t0 < sec:
            self.vel_pub.publish(t)
            self.spin_for(0.05)
        self.vel_pub.publish(Twist())
        self.spin_for(0.3)

    # ---- 撮影 ----
    def _fresh(self, attr, timeout=5.0):
        setattr(self, attr, None)
        t0 = time.time()
        while getattr(self, attr) is None and time.time() - t0 < timeout:
            self.spin_for(0.05)
        return getattr(self, attr)

    def snap(self, path):
        """カラーを png で保存する。"""
        import cv2
        m = self._fresh('arm_img')
        if m is None:
            self.get_logger().warn('アーム先端カメラの画像が来ない')
            return None
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, a[:, :, ::-1] if m.encoding == 'rgb8' else a)
        return path

    def snap_rgbd(self, prefix):
        """rs_snap.cpp と同じ書式（_color.png・_depth.pgm[mm, ビッグエンディアン]・_info.txt）で保存する。"""
        self.spin_for(0.5)            # 動いた直後の画像を避ける
        if self.snap(prefix + '_color.png') is None:
            return None
        m = self._fresh('arm_depth')
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).astype(float)
        d[~np.isfinite(d)] = 0.0
        mm = np.clip(d * 1000 + 0.5, 0, 65535).astype('>u2')
        with open(prefix + '_depth.pgm', 'wb') as f:
            f.write(f'P5\n{m.width} {m.height}\n65535\n'.encode() + mm.tobytes())
        if self.cam_k:
            fx, cx, fy, cy = self.cam_k[0], self.cam_k[2], self.cam_k[4], self.cam_k[5]
        else:                          # camera_info が来ないときは xacro の画角(1.518rad)から
            fx = fy = (m.width / 2) / math.tan(1.518 / 2)
            cx, cy = m.width / 2, m.height / 2
        with open(prefix + '_info.txt', 'w') as f:
            f.write(f'color {m.width}x{m.height}  fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}\n'
                    'depth_scale=0.001000 m/unit\n(gazebo)\n')
        return prefix

    # ---- 本体 ----
    def prepare(self):
        """録画の前に呼ぶ: 体を上げ切り（前の回が伏せたまま終わっていても同じ状態から始める）、
        アームを従来の収納にして、REST で止まって立つ。"""
        self.mode('STAND')
        self.spin_for(0.5)
        self.body_z(+1, 8.5)
        self.arm_to(OLD_STOW, speed=math.radians(90))
        self.mode('REST')
        self.spin_for(1.5)
        return 0

    def run(self, hold):
        poses = json.load(open(POSES))
        post = poses['postures']['lie']
        fast = post['deploy_fast']
        slow_n = len(post['deploy']) - 1
        bracket = post['bracket']
        os.makedirs(AIM_DIR, exist_ok=True)

        # ① 収納（止まったまま。REST で立って止まる。歩くのは TROT にしてから）
        self.status('① 止まったまま: 従来の収納は肘が左後ろ（angle0=68.7）')
        self.arm_to(OLD_STOW, speed=math.radians(90))
        self.spin_for(3.0)
        self.status('① 止まったまま収納を真後ろへ（angle0=0、腕を機体の軸に沿わせる）')
        self.arm_to(poses['stow'], speed=math.radians(30))
        self.spin_for(2.0)
        self.status('① 収納し終えてから歩き出す（接近中はアームを真後ろに固定）')
        self.mode('TROT')
        self.spin_for(2.0)
        for _ in range(3):
            self.enable_pub.publish(Bool(data=True))
            self.spin_for(0.1)
        stow_msg = Float64MultiArray(data=list(self.arm_cmd))
        t0 = time.time()
        while self.approach_result is None and time.time() - t0 < 150:
            self.arm_pub.publish(stow_msg)      # 収納の指令を送り続けて固定する
            self.spin_for(0.2)
        if self.joints is not None:
            dev = max(abs(math.degrees(a - b)) for a, b in zip(self.joints[:6], stow_msg.data[:6]))
            self.get_logger().info(f'接近中のアームのずれ（到着時）: 最大 {dev:.1f}度')
        if self.approach_result is None or not self.approach_result[0]:
            self.status(f'接近が到達で終わらなかった: {self.approach_result}')
            return 1
        self.status('① 到着')
        self.spin_for(1.5)

        # ② 伏せて速い展開
        self.status('② 伏せる')
        self.mode('STAND')
        self.spin_for(0.5)
        # 8.5秒で胴体が床から約0.13m。モデルの伏せ（0.125m）に合わせる（6秒だと0.17mで、実機より高い）
        self.body_z(-1, 8.5)
        self.spin_for(1.0)
        t0 = time.time()
        for k, p in enumerate(fast[1:], 1):
            moved = [f'angle{i}' for i in range(6) if abs(p[i] - fast[k - 1][i]) > 1e-6]
            how = f'{"・".join(moved)} を同時に' if len(moved) > 1 else f'{moved[0]} のみ'
            self.status(f'② 撮影姿勢へ {k}/{len(fast) - 1} 段目（{how}）従来は{slow_n}段')
            self.arm_to(p)
        self.status(f'② 撮影姿勢に到着: {len(fast) - 1}段・{time.time() - t0:.1f}秒')
        self.spin_for(1.5)

        # ③ 撮影位置の補正（inspect_run.py --aim-correct と同じ correct_loop）
        ref = list(fast[-1])
        corr_path = [ref]

        def take(k):
            self.status(f'③ 試し撮り {k + 1} 回目（深度つき）→ 隅肉の根元を探す')
            return self.snap_rgbd(f'{AIM_DIR}/probe{k + 1}')

        def move(q):
            self.status(f'③ 根元が中央・0.22m に来るようアームを直す: {" ".join(f"{v:g}" for v in q)}')
            self.arm_to(q, speed=math.radians(25))
            corr_path.append(list(q))

        def log(msg):
            self.status('③' + msg.strip().replace('[補正', '').replace(']', '', 1))
            self.spin_for(2.5)       # 重ね画像を見せる

        q, ok = aim_correct.correct_loop(take, move, ref, ref, dist=0.22, lying=True, log=log)
        hold_q = list(q)
        self.spin_for(1.0)

        # 撮影（手首を振って3枚。振り幅は基準姿勢からの差で）
        name = time.strftime('sim_%m%d_%H%M%S')
        j = bracket['joint']
        for k, ang in enumerate(bracket['angles']):
            p = list(hold_q)
            p[j] = round(hold_q[j] + (ang - ref[j]), 1)
            self.status(f'④ 撮影 {k + 1}/3（手首 {p[j]:g}度）')
            self.arm_to(p)
            self.spin_for(0.8)
            self.snap(f'{SHOT_DIR}/{name}_{k + 1}.png')
            self.spin_for(0.6)
        self.arm_to(hold_q)
        self.spin_for(hold)

        # 収納: 補正を逆にたどり、速い経路を逆に
        self.status('⑤ 収納（補正を戻してから、3段の経路を逆に）')
        for p in reversed(corr_path[:-1]):
            self.arm_to(p)
        for p in reversed(fast[:-1]):
            self.arm_to(p)

        self.status('⑥ 起立')
        self.body_z(+1, 8.5)
        self.mode('REST')
        self.spin_for(0.5)
        self.mode('TROT')
        self.spin_for(3.5)
        self.status('完了')
        self.spin_for(2.0)
        return 0


def main():
    prep = '--prepare' in sys.argv
    args = [a for a in sys.argv[1:] if a != '--prepare']
    hold = float(args[0]) if args else 2.0
    rclpy.init()
    n = Demo()
    try:
        return n.prepare() if prep else n.run(hold)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
