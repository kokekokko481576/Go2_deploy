#!/usr/bin/env python3
"""Gazebo デモの録画（横一線スイープ版。sweep_demo.py と組む。2026-10-09）。

recorder_1007.py の写し。違いは段階の名前（④ 横一線スイープ）と、④ の間は右下に
最新の検出の重ね画像（/tmp/sweep_aim/sweep*_probe*_root.png）を出すところ。

    左上（大）: 全体カメラ /overview_cam（spawn_scene.sh が置いた固定カメラ）
    右上     : 前方カメラ /robot1/color/image_raw ＋ マーカー検知の枠・ID・距離
    右下     : アーム先端カメラ（④ の間は検出の重ね画像）
    下       : 流れの段階・字幕（/demo_status）・経過時間

   python3 recorder_sweep.py /tmp/sweep.mp4
"""
import glob
import signal
import sys
import time

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from PIL import Image as PImage, ImageDraw, ImageFont
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

FONT = '/tmp/fonts/NotoSansCJK-Regular.ttc'
W, H, FPS = 1920, 1080, 15
STEPS = ['① 接近', '② 伏せる', '③ 撮影姿勢へ', '④ 横一線スイープ', '⑤ 収納', '⑥ 起立']
BG = (24, 30, 42)
ACCENT = (80, 170, 240)


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    if m.encoding == 'rgb8':
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a[:, :, :3])


class Rec(Node):
    def __init__(self, out):
        super().__init__('sweep_recorder')
        self.imgs = {}
        self.det = None
        self.det_t = 0.0
        self.dist = None
        self.status = '準備中'
        self.done_t = None
        self.t0 = time.time()
        self.writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'avc1'), FPS, (W, H))
        self.font_l = ImageFont.truetype(FONT, 40)
        self.font_m = ImageFont.truetype(FONT, 26)
        self.font_s = ImageFont.truetype(FONT, 20)
        for key, topic in (('ov', '/overview_cam'), ('front', '/robot1/color/image_raw'), ('arm', '/robot1/arm_cam/image')):
            self.create_subscription(Image, topic, lambda m, k=key: self.imgs.__setitem__(k, m), 2)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_det, 5)
        self.create_subscription(PoseStamped, '/marker_pose', self.on_pose, 5)
        self.create_subscription(String, '/demo_status', self.on_status, 10)
        self.create_timer(1.0 / FPS, self.frame)
        self.frames = 0

    def on_det(self, m):
        if m.detections:
            self.det = m.detections[0]
            self.det_t = time.time()

    def on_pose(self, m):
        p = m.pose.position
        self.dist = (p.x ** 2 + p.y ** 2 + p.z ** 2) ** 0.5

    def on_status(self, m):
        self.status = m.data
        if (m.data.startswith('=== まとめ') or m.data == '完了') and self.done_t is None:
            self.done_t = time.time()

    def panel(self, key, w, h):
        m = self.imgs.get(key)
        if m is None:
            return np.zeros((h, w, 3), np.uint8)
        img = to_bgr(m)
        sx, sy = w / img.shape[1], h / img.shape[0]
        img = cv2.resize(img, (w, h))
        if key == 'front' and self.det is not None and time.time() - self.det_t < 0.5:
            pts = np.array([[c.x * sx, c.y * sy] for c in self.det.corners], np.int32)
            cv2.polylines(img, [pts], True, (60, 220, 60), 3)
            cx, cy = int(self.det.centre.x * sx), int(self.det.centre.y * sy)
            cv2.circle(img, (cx, cy), 5, (60, 220, 60), -1)
        return img

    def aim_overlay(self):
        """④ の間: 一番新しい検出の重ね画像。無ければ None。"""
        fs = sorted(glob.glob('/tmp/sweep_aim/sweep*_probe*_root.png'))
        if not fs:
            return None
        img = cv2.imread(fs[-1])
        return None if img is None else cv2.resize(img, (640, 480))

    def frame(self):
        canvas = np.full((H, W, 3), BG[::-1], np.uint8)
        canvas[0:720, 0:1280] = self.panel('ov', 1280, 720)
        canvas[40:520, 1280:1920] = self.panel('front', 640, 480)
        overlay = self.aim_overlay() if self.status.startswith('④') or self.status.startswith('  点') else None
        canvas[560:1040, 1280:1920] = overlay if overlay is not None else self.panel('arm', 640, 480)
        im = PImage.fromarray(canvas[:, :, ::-1])
        d = ImageDraw.Draw(im)
        detected = self.det is not None and time.time() - self.det_t < 0.5
        d.text((1292, 6), '前方カメラ：マーカー検知', font=self.font_s, fill=(230, 235, 240))
        if detected:
            txt = f'検出中 ID {self.det.id}' + (f'  距離 {self.dist:.2f} m' if self.dist else '')
            d.text((1292 + 300, 6), txt, font=self.font_s, fill=(120, 230, 120))
        else:
            d.text((1292 + 300, 6), '未検出', font=self.font_s, fill=(200, 200, 200))
        d.text((1292, 526), '深度で見つけた根元（緑）と狙う点（赤）' if overlay is not None else 'アーム先端カメラ（撮影）',
               font=self.font_s, fill=(230, 235, 240))
        cur = next((i for i, s in enumerate(STEPS) if self.status.startswith(s[0])), -1)
        if cur < 0 and self.status.startswith('  点'):
            cur = 3   # スイープ各点の細かいログは ④ 扱い
        x = 30
        for i, s in enumerate(STEPS):
            on = i == cur
            done = cur > i or self.done_t is not None
            w = 198
            d.rounded_rectangle((x, 745, x + w, 800), 8,
                                fill=ACCENT if on else ((60, 90, 120) if done else (48, 56, 70)))
            d.text((x + 12, 757), s, font=self.font_s, fill=(255, 255, 255) if (on or done) else (150, 158, 170))
            x += w + 10
        st = self.status if len(self.status) <= 44 else self.status[:44] + '…'
        d.text((30, 830), st, font=self.font_l, fill=(255, 255, 255))
        d.text((30, 900), f'経過 {time.time() - self.t0:5.1f} 秒', font=self.font_m, fill=(170, 180, 195))
        d.text((30, 960), 'Gazebo シミュレーション／実機と同じ接近制御・アーム姿勢（inspect_poses.json）・補正（aim_correct.py）',
               font=self.font_s, fill=(150, 160, 175))
        d.text((30, 992), '横一線撮影: ビード方向に4点、各点で深度から根元を見つけてアームを直してから撮る',
               font=self.font_s, fill=(150, 160, 175))
        self.writer.write(np.asarray(im)[:, :, ::-1])
        self.frames += 1
        if self.done_t is not None and time.time() - self.done_t > 3.0:
            raise SystemExit

    def close(self):
        self.writer.release()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/sweep.mp4'
    rclpy.init()
    n = Rec(out)
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.close()
        print(f'[recorder] {n.frames} フレーム（{n.frames / FPS:.1f} 秒）を書きました: {out}', flush=True)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
