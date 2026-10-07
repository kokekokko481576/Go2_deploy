#!/usr/bin/env python3
"""Gazebo デモの録画（10/7 版。demo_1007.py と組む）。複数のカメラと字幕を1枚にまとめて mp4 にする。

recorder.py（9/29）からの違い: 段階の名前、③ の間は右下に検出の重ね画像（/tmp/demo_aim/probeN_root.png）、
撮り終えたら右下に「補正前の試し撮り」と「補正後」を並べる。

    左上（大）: 全体カメラ /overview_cam（spawn_scene.sh が置いた固定カメラ）
    右上     : 前方カメラ /robot1/color/image_raw ＋ マーカー検知の枠・ID・距離（/detections・marker_pose）
    右下     : アーム先端カメラ /robot1/arm_cam/image
    下       : 流れの段階（今の段階を強調）・字幕（/demo_status）・経過時間

画面録画ではなくトピックから作るので、窓の重なりや端末の表示に左右されない。
Ctrl+C（SIGINT）か /demo_status が「完了」になって数秒で書き終える。
   python3 recorder.py /tmp/demo.mp4
"""
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
STEPS = ['① 収納・接近', '② 伏せ・展開', '③ 位置の補正', '④ 撮影 3枚', '⑤ 収納', '⑥ 起立']
BG = (24, 30, 42)
ACCENT = (80, 170, 240)


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    if m.encoding == 'rgb8':
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a[:, :, :3])


class Rec(Node):
    def __init__(self, out):
        super().__init__('demo_recorder')
        self.imgs = {}
        self.det = None
        self.det_t = 0.0
        self.dist = None
        self.status = '準備中'
        self.done_t = None
        self.t0 = time.time()
        # H.264（PowerPoint・LibreOffice で再生しやすい）
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
        if m.data == '完了' and self.done_t is None:
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

    def shots(self):
        """demo_flow.py が保存した写真（/tmp/demo_shots）を、縦横比を保って横に3枚並べる。まだ無ければ None。"""
        import glob
        fs = sorted(glob.glob('/tmp/demo_shots/*.png'))[-3:]
        if len(fs) < 3:
            return None
        if getattr(self, '_shots_key', None) != fs:
            panel = np.full((480, 640, 3), BG[::-1], np.uint8)
            tw, th = 204, 153                       # 4:3。3枚＋すき間8px×2＋左右の余白6px で 640 に収める
            y0 = (480 - th) // 2
            for k, f in enumerate(fs):
                x0 = 6 + k * (tw + 8)
                panel[y0:y0 + th, x0:x0 + tw] = cv2.resize(cv2.imread(f), (tw, th))
            self._shots_img = panel
            self._shots_key = fs
        return self._shots_img

    def aim_overlay(self):
        """③ の間: 一番新しい検出の重ね画像。無ければ None。"""
        import glob
        fs = sorted(glob.glob('/tmp/demo_aim/probe*_root.png'))
        if not fs:
            return None
        img = cv2.imread(fs[-1])
        return None if img is None else cv2.resize(img, (640, 480))

    def before_after(self):
        """撮り終えたら: 補正前の試し撮り（重ね画像）と、補正後の基準の1枚を並べる。"""
        import glob
        before = '/tmp/demo_aim/probe1_root.png'
        after = sorted(glob.glob('/tmp/demo_shots/*_2.png'))
        b = cv2.imread(before)
        if b is None or not after:
            return None
        a = cv2.imread(after[-1])
        panel = np.full((480, 640, 3), BG[::-1], np.uint8)
        tw, th = 312, 234
        y0 = (480 - th) // 2
        panel[y0:y0 + th, 6:6 + tw] = cv2.resize(b, (tw, th))
        panel[y0:y0 + th, 322:322 + tw] = cv2.resize(a, (tw, th))
        return panel

    def frame(self):
        canvas = np.full((H, W, 3), BG[::-1], np.uint8)
        canvas[0:720, 0:1280] = self.panel('ov', 1280, 720)
        canvas[40:520, 1280:1920] = self.panel('front', 640, 480)
        late = self.status.startswith(('⑤', '⑥', '完了'))
        shots = self.before_after() if late else None
        overlay = self.aim_overlay() if self.status.startswith('③') and ('根元は' in self.status or '合っている' in self.status
                                                                         or '直す' in self.status or '見つからない' in self.status) else None
        canvas[560:1040, 1280:1920] = (shots if shots is not None else
                                       overlay if overlay is not None else self.panel('arm', 640, 480))
        im = PImage.fromarray(canvas[:, :, ::-1])
        d = ImageDraw.Draw(im)
        detected = self.det is not None and time.time() - self.det_t < 0.5
        d.text((1292, 6), '前方カメラ：マーカー検知', font=self.font_s, fill=(230, 235, 240))
        if detected:
            txt = f'検出中 ID {self.det.id}' + (f'  距離 {self.dist:.2f} m' if self.dist else '')
            d.text((1292 + 300, 6), txt, font=self.font_s, fill=(120, 230, 120))
        else:
            d.text((1292 + 300, 6), '未検出', font=self.font_s, fill=(200, 200, 200))
        d.text((1292, 526), '補正前と補正後' if shots is not None else
               '深度で見つけた根元（緑）と狙う点（赤）' if overlay is not None else 'アーム先端カメラ（撮影）',
               font=self.font_s, fill=(230, 235, 240))
        if shots is not None:
            for k, lab in enumerate(('補正前の試し撮り', '補正後（基準の1枚）')):
                d.text((1280 + 6 + k * 316, 560 + 123 + 234 + 10), lab, font=self.font_s, fill=(200, 208, 220))
        # 流れの段階
        cur = next((i for i, s in enumerate(STEPS) if self.status.startswith(s[0])), -1)
        x = 30
        for i, s in enumerate(STEPS):
            on = i == cur
            done = cur > i or self.status == '完了'
            w = 198
            d.rounded_rectangle((x, 745, x + w, 800), 8,
                                fill=ACCENT if on else ((60, 90, 120) if done else (48, 56, 70)))
            d.text((x + 12, 757), s, font=self.font_s, fill=(255, 255, 255) if (on or done) else (150, 158, 170))
            x += w + 10
        st = self.status if len(self.status) <= 44 else self.status[:44] + '…'
        d.text((30, 830), st, font=self.font_l, fill=(255, 255, 255))
        d.text((30, 900), f'経過 {time.time() - self.t0:5.1f} 秒', font=self.font_m, fill=(170, 180, 195))
        d.text((30, 960), 'Gazebo シミュレーション ／ 実機と同じ接近制御・同じアーム姿勢（inspect_poses.json）・同じ補正（aim_correct.py）',
               font=self.font_s, fill=(150, 160, 175))
        d.text((30, 992), '① 収納を真後ろへ　② 3段で撮影姿勢へ　③ 深度で根元を見つけてアームを直す（部材はわざと約7cmずらした）',
               font=self.font_s, fill=(150, 160, 175))
        self.writer.write(np.asarray(im)[:, :, ::-1])
        self.frames += 1
        if self.done_t is not None and time.time() - self.done_t > 3.0:
            raise SystemExit

    def close(self):
        self.writer.release()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/demo.mp4'
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
