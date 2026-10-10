#!/usr/bin/env python3
"""Gazebo（go2-sim コンテナ内、Jazzy）で、実機デモと同じ流れを動かす（2026-09-29、作業報告の動画用）。

    マーカー接近（approach_node） → 伏せる → アーム展開 → 手首を振って3枚撮影 → 収納 → 起立

実機の inspect_run.py と同じ姿勢（/marker_detection/tools/inspect_poses.json の lie）を使う。
実機との違い（sim の都合）:
  - 伏せる: sim の脚制御には伏せる命令がないので、STAND モードで体を約12cmゆっくり下げる
    （robot_mode=STAND ＋ cmd_vel.linear.z。cmd_vel_pub が z をそのまま robot_velocity へ渡す）
  - アーム: D1 SDK の角度[度]を、d1_arm_controller（関節位置の直接指令）へ rad で送る。
    SDK の angle0〜5 と URDF の joint1〜6 は符号・ゼロ点とも一致（2026-09-23/28 実機で確認）。
    直接指令だと一瞬で飛ぶので、実機に近い速さ（40度/秒）で少しずつ送る
  - 撮影: アーム先端カメラ（/robot1/arm_cam/image）の画像を /tmp/demo_shots/ に保存
  - 温度の監視はしない
今どの段階かを /demo_status（std_msgs/String）に出す。録画ノードが字幕に使う。
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
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from quadropted_msgs.msg import RobotModeCommand

POSES = '/marker_detection/tools/inspect_poses.json'
ARM_JOINTS = ['d1_joint1', 'd1_joint2', 'd1_joint3', 'd1_joint4', 'd1_joint5', 'd1_joint6', 'd1_joint_l', 'd1_joint_r']
ARM_SPEED = math.radians(40.0)   # rad/s
SHOT_DIR = '/tmp/demo_shots'


class Demo(Node):
    def __init__(self):
        super().__init__('demo_flow')
        self.status_pub = self.create_publisher(String, '/demo_status', 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, '/robot1/d1_arm_controller/commands', 10)
        self.mode_pub = self.create_publisher(RobotModeCommand, '/robot1/robot_mode', 10)
        self.vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/marker_approach_node/enable', 10)
        self.joints = None
        self.arm_img = None
        self.approach_result = None
        self.approach_state = None
        self.create_subscription(JointState, '/robot1/joint_states', self.on_js, 10)
        self.create_subscription(Image, '/robot1/arm_cam/image', self.on_img, 2)
        self.create_subscription(Log, '/rosout', self.on_log, 50)
        self.create_subscription(String, '/marker_approach_node/state', self.on_state, 10)
        self.arm_cmd = None      # 直近に送った 8 関節の指令[rad]

    # ---- 購読 ----
    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM_JOINTS):
            self.joints = [d[j] for j in ARM_JOINTS]

    def on_img(self, m):
        self.arm_img = m

    def on_log(self, m):
        if m.name.endswith('marker_approach_node') and '停止しました' in m.msg:
            self.approach_result = ('到達しました' in m.msg, m.msg)

    def on_state(self, m):
        self.approach_state = m.data

    # ---- 共通 ----
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
        """SDK の angle0〜5[度] の姿勢へ、関節ごとに一定速度で動かす（最も遠い関節に合わせて同時に着く）。"""
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
            a = a * a * (3 - 2 * a)      # 始まりと終わりをなめらかに
            cmd = [s + (t - s) * a for s, t in zip(start, target)]
            self.arm_pub.publish(Float64MultiArray(data=cmd))
            self.arm_cmd = cmd
            self.spin_for(1.0 / 30)
        self.spin_for(0.3)

    # ---- 姿勢 ----
    def mode(self, name):
        for _ in range(3):
            self.mode_pub.publish(RobotModeCommand(mode=name, robot_id=1))
            self.spin_for(0.1)

    def body_z(self, direction, sec):
        """STAND モードで体を上下させる（direction=-1 で下げる）。"""
        t = Twist()
        t.linear.z = float(direction)
        t0 = time.time()
        while time.time() - t0 < sec:
            self.vel_pub.publish(t)
            self.spin_for(0.05)
        self.vel_pub.publish(Twist())
        self.spin_for(0.3)

    # ---- 撮影 ----
    def snap(self, name):
        self.arm_img = None
        t0 = time.time()
        while self.arm_img is None and time.time() - t0 < 5:
            self.spin_for(0.05)
        if self.arm_img is None:
            self.get_logger().warn('アーム先端カメラの画像が来ない')
            return
        m = self.arm_img
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        import cv2
        os.makedirs(SHOT_DIR, exist_ok=True)
        cv2.imwrite(f'{SHOT_DIR}/{name}.png', a[:, :, ::-1] if m.encoding == 'rgb8' else a)

    # ---- 本体 ----
    def run(self, hold):
        poses = json.load(open(POSES))
        post = poses['postures']['lie']
        seq = post['deploy']
        bracket = post.get('bracket')

        self.status('準備: アームを収納姿勢へ')
        self.arm_to(poses['stow'], speed=math.radians(90))
        self.spin_for(1.0)

        self.status('① マーカー検知（前方カメラで AprilTag を見つける）')
        self.spin_for(2.5)
        self.status('① 接近（視線接近・マーカーの正面 0.65m へ）')
        for _ in range(3):
            self.enable_pub.publish(Bool(data=True))
            self.spin_for(0.1)
        t0 = time.time()
        while self.approach_result is None and time.time() - t0 < 150:
            self.spin_for(0.2)
        if self.approach_result is None or not self.approach_result[0]:
            self.status(f'接近が到達で終わらなかった: {self.approach_result}')
            return 1
        self.status('① 到着（マーカーの正面 0.65m）')
        self.spin_for(1.5)

        self.status('② 伏せる')
        self.mode('STAND')
        self.spin_for(0.5)
        self.body_z(-1, 6.0)
        self.spin_for(1.0)

        self.status('③ アームを撮影姿勢へ')
        for p in seq[1:]:
            self.arm_to(p)

        name = time.strftime('sim_%m%d_%H%M%S')
        if bracket:
            j = bracket['joint']
            for k, ang in enumerate(bracket['angles']):
                self.status(f'④ 撮影 {k + 1}/{len(bracket["angles"])}（手首 {ang:g}度）')
                p = list(seq[-1])
                p[j] = ang
                self.arm_to(p)
                self.spin_for(0.8)
                self.snap(f'{name}_{k + 1}')
                self.spin_for(0.8)
            self.arm_to(seq[-1])
        else:
            self.status('④ 撮影')
            self.snap(name)
        self.spin_for(hold)

        self.status('⑤ アームを収納')
        for p in reversed(seq[:-1]):
            self.arm_to(p)

        self.status('⑥ 起立')
        self.body_z(+1, 6.0)
        self.mode('REST')
        self.spin_for(0.5)
        self.mode('TROT')
        self.spin_for(3.5)      # 立ち上がり切るまで映す
        self.status('完了')
        self.spin_for(2.0)
        return 0


def main():
    hold = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    rclpy.init()
    n = Demo()
    try:
        return n.run(hold)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
