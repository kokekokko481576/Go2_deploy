#!/usr/bin/env python3
"""Gazeboで、横一線スイープ+aim_correctの統合を実際の深度画像で確かめる（2026-10-09）。

demo_1007.py の③（aim_correct.correct_loop を実際のGazebo深度画像で動かす部分）と同じやり方を、
inspect_poses.json の lie.sweep（4点）に適用する。各点で試し撮り→根元を探す→直す→合えば本撮影、を行う。
inspect_run.py --sweep --aim-correct と同じロジック（アームの先・ROS 以外は共通）。

    python3 sweep_demo.py
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

sys.path.insert(0, '/inspection')
import aim_correct  # noqa: E402

POSES = '/inspection/inspect_poses.json'
ARM_JOINTS = ['d1_joint1', 'd1_joint2', 'd1_joint3', 'd1_joint4', 'd1_joint5', 'd1_joint6', 'd1_joint_l', 'd1_joint_r']
ARM_SPEED = math.radians(40.0)
SHOT_DIR = '/tmp/sweep_shots'
AIM_DIR = '/tmp/sweep_aim'


class SweepDemo(Node):
    def __init__(self):
        super().__init__('sweep_demo')
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
        self.create_subscription(CameraInfo, '/robot1/arm_cam/camera_info',
                                 lambda m: setattr(self, 'cam_k', list(m.k)), 2)
        self.create_subscription(Log, '/rosout', self.on_log, 50)
        self.arm_cmd = None

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

    def arm_to(self, sdk_deg, speed=ARM_SPEED):
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

    def _fresh(self, attr, timeout=5.0):
        setattr(self, attr, None)
        t0 = time.time()
        while getattr(self, attr) is None and time.time() - t0 < timeout:
            self.spin_for(0.05)
        return getattr(self, attr)

    def snap(self, path):
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
        """rs_snap.cpp と同じ書式（_color.png・_depth.pgm[mm,ビッグエンディアン]・_info.txt）で保存する。"""
        self.spin_for(0.5)
        if self.snap(prefix + '_color.png') is None:
            return None
        m = self._fresh('arm_depth')
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).astype(float)
        d[~np.isfinite(d)] = 0.0
        mm = np.clip(d * 1000 + 0.5, 0, 65535).astype('>u2')
        os.makedirs(os.path.dirname(prefix), exist_ok=True)
        with open(prefix + '_depth.pgm', 'wb') as f:
            f.write(f'P5\n{m.width} {m.height}\n65535\n'.encode() + mm.tobytes())
        if self.cam_k:
            fx, cx, fy, cy = self.cam_k[0], self.cam_k[2], self.cam_k[4], self.cam_k[5]
        else:
            fx = fy = (m.width / 2) / math.tan(1.518 / 2)
            cx, cy = m.width / 2, m.height / 2
        with open(prefix + '_info.txt', 'w') as f:
            f.write(f'color {m.width}x{m.height}  fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}\n'
                    'depth_scale=0.001000 m/unit\n(gazebo)\n')
        return prefix

    def run(self):
        poses = json.load(open(POSES))
        post = poses['postures']['lie']
        seq = post['deploy']
        sweep = post['sweep']

        self.status('準備: アームを収納姿勢へ')
        self.arm_to(poses['stow'], speed=math.radians(90))
        self.spin_for(1.0)

        self.status('① マーカー接近（視線接近・マーカーの正面0.65mへ）')
        self.mode('TROT')
        self.spin_for(2.0)
        for _ in range(3):
            self.enable_pub.publish(Bool(data=True))
            self.spin_for(0.1)
        t0 = time.time()
        while self.approach_result is None and time.time() - t0 < 45:
            self.spin_for(0.2)
        if self.approach_result is not None and not self.approach_result[0]:
            self.status(f'接近が到達で終わらなかった: {self.approach_result}')
            return 1
        if self.approach_result is None:
            self.status('** 成功ログが来なかった（approach_nodeのロガー不具合の可能性）。位置の安定を仮定して進める **')
        else:
            self.status('① 到着')
        self.spin_for(1.5)

        self.status('② 伏せる')
        self.mode('STAND')
        self.spin_for(0.5)
        self.body_z(-1, 8.5)
        self.spin_for(1.0)

        self.status('③ アームを撮影姿勢(基準=deploy[-1])へ')
        for p in seq[1:]:
            self.arm_to(p)
        hold = list(seq[-1])

        self.status(f'④ 横一線スイープ {len(sweep["poses"])}点。各点で深度の根元に合わせてから撮る')
        results = []
        name = time.strftime('sweep_%m%d_%H%M%S')
        for k, (dx, p) in enumerate(zip(sweep['dx'], sweep['poses']), 1):
            self.status(f'  点{k} dx={dx:+.3f}へ移動')
            self.arm_to(p)

            corr_path = [list(p)]

            def take(i, k=k):
                self.status(f'  点{k} 試し撮り{i + 1}回目')
                return self.snap_rgbd(f'{AIM_DIR}/sweep{k}_probe{i + 1}')

            def move(q, k=k):
                self.status(f'  点{k} 補正: {" ".join(f"{v:g}" for v in q)}')
                self.arm_to(q, speed=math.radians(25))
                corr_path.append(list(q))

            def log(msg, k=k):
                self.status(f'  点{k}' + msg.strip().replace('[補正', '').replace(']', '', 1))

            q, ok = aim_correct.correct_loop(take, move, list(p), list(p), dist=sweep['dist'],
                                             lying=True, log=log)
            self.status(f'  点{k}: {"合った" if ok else "合わなかった(そのまま撮る)"}')
            self.snap(f'{SHOT_DIR}/{name}_{k}_dx{dx:+.3f}.png')
            results.append((dx, ok, q))
            # 補正を戻してから次の点・最後は基準へ
            for pp in reversed(corr_path[:-1]):
                self.arm_to(pp)

        self.status('⑤ 基準姿勢へ戻して収納')
        self.arm_to(hold)
        for p in reversed(seq[:-1]):
            self.arm_to(p)
        self.status('⑥ 起立')
        self.body_z(+1, 8.5)
        self.mode('REST')
        self.spin_for(0.5)
        self.mode('TROT')
        self.spin_for(2.0)

        self.status('=== まとめ ===')
        for dx, ok, q in results:
            self.status(f'  dx={dx:+.3f}: {"合った" if ok else "合わなかった"}  最終角度 {[round(v,1) for v in q]}')
        return 0


def main():
    rclpy.init()
    node = SweepDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
