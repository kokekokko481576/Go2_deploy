#!/usr/bin/env python3
"""Gazeboナビゲーションデモ(地図作成→自己位置推定→経路追従)の録画(2026-11-13)。

recorder.py(マーカー接近+アーム展開デモ用)と同じ「画面録画ではなくトピックから
組み立てる」方式。複数視点を1枚にまとめてmp4にする:

    左    : 作成済み地図(occupancy grid) + 自己位置(AMCL) + 計画経路(/plan) + 走行軌跡
    右上  : ロボット前方カメラ(/robot1/color/image_raw)
    右下  : 状態パネル(現在地・目標・残り距離・経過時間・フェーズ)

使い方(arbeit-ros2コンテナ内):
    python3 recorder_nav.py /tmp/nav_demo.mp4
Ctrl+C で録画終了・書き出し。
"""
import math
import signal
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from PIL import Image as PImage, ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# map_server(/go2_localization/map)はTRANSIENT_LOCALでラッチ配信しているため、
# 既定(VOLATILE)のサブスクライバでは受け取れず地図パネルが空白のままになる
# (2026-11-13に気付いて修正)。/planはVOLATILEなので既定のままでよい
# (一度TRANSIENT_LOCALに揃えて逆にQoS不一致で受信できなくなった失敗から学んだ)
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
W, H, FPS = 1920, 1080, 15
BG = (24, 30, 42)
MAP_PANEL = (0, 0, 640, 1080)
CAM_PANEL = (640, 0, 1920, 650)
INFO_PANEL = (640, 650, 1920, 1080)

GOAL_REACHED_DIST = 0.35


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    if m.encoding == 'rgb8':
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a[:, :, :3])


class NavRec(Node):
    def __init__(self, out):
        super().__init__('nav_demo_recorder')
        self.cam_img = None
        self.map_msg = None
        self.map_base_bgr = None   # 地図をBGR画像化してキャッシュ(毎フレーム再計算しない)
        self.map_scale = 1.0
        self.map_off = (0, 0)      # MAP_PANEL内でのオフセット(letterbox用)
        self.pose = None           # (x, y, yaw) 最新のAMCL姿勢
        self.trail = []            # 走行軌跡(map座標のリスト)
        self.path_pts = []         # 直近の/plan(map座標のリスト)
        self.goal = None           # 直近の/goal_pose (x, y)
        self.goal_t = 0.0
        self.t0 = time.time()
        self.writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'avc1'), FPS, (W, H))
        self.font_l = ImageFont.truetype(FONT, 34)
        self.font_m = ImageFont.truetype(FONT, 24)
        self.font_s = ImageFont.truetype(FONT, 18)

        self.create_subscription(Image, '/robot1/color/image_raw', self._on_cam, 2)
        self.create_subscription(OccupancyGrid, '/go2_localization/map', self._on_map, MAP_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, '/go2_localization/amcl_pose', self._on_pose, 10)
        self.create_subscription(Path, '/plan', self._on_plan, 5)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_timer(1.0 / FPS, self.frame)
        self.frames = 0

    def _on_cam(self, m):
        self.cam_img = m

    def _on_map(self, m):
        self.map_msg = m
        self._render_map_base()

    def _on_pose(self, m):
        p = m.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)
        if not self.trail or math.hypot(
                p.position.x - self.trail[-1][0], p.position.y - self.trail[-1][1]) > 0.03:
            self.trail.append((p.position.x, p.position.y))

    def _on_plan(self, m):
        self.path_pts = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]

    def _on_goal(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y)
        self.goal_t = time.time()
        self.trail = []  # 新しいゴールごとに軌跡をリセット(見やすさのため)

    def _render_map_base(self):
        m = self.map_msg
        grid = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        color = np.full((m.info.height, m.info.width, 3), 220, np.uint8)   # 未知=グレー
        color[grid == 0] = (255, 255, 255)     # 空きスペース=白
        color[grid > 50] = (40, 40, 40)        # 障害物=黒っぽい
        color = np.flipud(color)               # ROSの行0=y最小(下) → 画像の行0=上 に反転
        panel_w = MAP_PANEL[2] - MAP_PANEL[0]
        panel_h = MAP_PANEL[3] - MAP_PANEL[1]
        scale = min(panel_w / m.info.width, panel_h / m.info.height) * 0.92
        new_w, new_h = int(m.info.width * scale), int(m.info.height * scale)
        resized = cv2.resize(color, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        self.map_base_bgr = resized[:, :, ::-1]  # RGBのつもりで作ったのでBGRへ
        self.map_scale = scale
        self.map_off = ((panel_w - new_w) // 2, (panel_h - new_h) // 2)

    def _world_to_panel(self, x, y):
        """map座標(x,y) -> MAP_PANEL内のピクセル座標"""
        m = self.map_msg
        col = (x - m.info.origin.position.x) / m.info.resolution
        ros_row = (y - m.info.origin.position.y) / m.info.resolution
        disp_row = m.info.height - 1 - ros_row
        px = int(col * self.map_scale) + self.map_off[0]
        py = int(disp_row * self.map_scale) + self.map_off[1]
        return px, py

    def _draw_map_panel(self, canvas):
        x0, y0, x1, y1 = MAP_PANEL
        panel = np.full((y1 - y0, x1 - x0, 3), BG[::-1], np.uint8)
        if self.map_base_bgr is not None:
            h, w = self.map_base_bgr.shape[:2]
            ox, oy = self.map_off
            panel[oy:oy + h, ox:ox + w] = self.map_base_bgr
            if self.map_msg is not None:
                if len(self.path_pts) >= 2:
                    pts = np.array([self._world_to_panel(x, y) for x, y in self.path_pts], np.int32)
                    cv2.polylines(panel, [pts], False, (230, 170, 60), 3)  # 計画経路: 青系
                if len(self.trail) >= 2:
                    pts = np.array([self._world_to_panel(x, y) for x, y in self.trail], np.int32)
                    cv2.polylines(panel, [pts], False, (90, 220, 90), 2)  # 走行軌跡: 緑
                if self.goal is not None:
                    gx, gy = self._world_to_panel(*self.goal)
                    cv2.drawMarker(panel, (gx, gy), (40, 210, 240), cv2.MARKER_STAR, 22, 2)
                if self.pose is not None:
                    px, py = self._world_to_panel(self.pose[0], self.pose[1])
                    yaw = self.pose[2]
                    tip = (int(px + 16 * math.cos(-yaw)), int(py + 16 * math.sin(-yaw)))
                    l = (int(px + 9 * math.cos(-yaw + 2.5)), int(py + 9 * math.sin(-yaw + 2.5)))
                    r = (int(px + 9 * math.cos(-yaw - 2.5)), int(py + 9 * math.sin(-yaw - 2.5)))
                    cv2.fillPoly(panel, [np.array([tip, l, r], np.int32)], (50, 60, 230))
        canvas[y0:y1, x0:x1] = panel

    def _draw_cam_panel(self, canvas):
        x0, y0, x1, y1 = CAM_PANEL
        w, h = x1 - x0, y1 - y0
        panel = np.full((h, w, 3), BG[::-1], np.uint8)
        if self.cam_img is not None:
            img = to_bgr(self.cam_img)
            ih, iw = img.shape[:2]
            scale = min(w / iw, h / ih)
            nw, nh = int(iw * scale), int(ih * scale)
            resized = cv2.resize(img, (nw, nh))
            ox, oy = (w - nw) // 2, (h - nh) // 2
            panel[oy:oy + nh, ox:ox + nw] = resized
        canvas[y0:y1, x0:x1] = panel

    def frame(self):
        canvas = np.full((H, W, 3), BG[::-1], np.uint8)
        self._draw_map_panel(canvas)
        self._draw_cam_panel(canvas)

        im = PImage.fromarray(canvas[:, :, ::-1])
        d = ImageDraw.Draw(im)
        d.text((16, 10), '作成済み地図 + 自己位置(AMCL) + 計画経路 + 走行軌跡',
               font=self.font_s, fill=(230, 235, 240))
        d.text((656, 662), 'ロボット前方カメラ', font=self.font_s, fill=(230, 235, 240))

        # 状態パネル
        ix0, iy0, ix1, iy1 = INFO_PANEL
        d.rectangle((ix0, iy0, ix1, iy1), fill=(18, 22, 32))
        y = iy0 + 20
        d.text((ix0 + 20, y), 'Go2 Gazebo デモ: 地図作成 → 自己位置推定 → 経路追従',
               font=self.font_l, fill=(255, 255, 255))
        y += 50
        if self.pose is not None:
            d.text((ix0 + 20, y),
                    f'現在地: x={self.pose[0]:+.2f}  y={self.pose[1]:+.2f}  '
                    f'yaw={math.degrees(self.pose[2]):+.0f}°',
                    font=self.font_m, fill=(220, 225, 230))
        y += 38
        if self.goal is not None:
            dist = math.hypot(self.goal[0] - (self.pose[0] if self.pose else 0),
                               self.goal[1] - (self.pose[1] if self.pose else 0))
            d.text((ix0 + 20, y), f'目標: x={self.goal[0]:+.2f}  y={self.goal[1]:+.2f}',
                    font=self.font_m, fill=(220, 225, 230))
            y += 38
            reached = dist < GOAL_REACHED_DIST
            phase = '到達！' if reached else '追従中'
            color = (120, 230, 120) if reached else (240, 200, 90)
            d.text((ix0 + 20, y), f'残り距離: {dist:.2f} m   状態: {phase}',
                    font=self.font_m, fill=color)
        else:
            y += 38
            d.text((ix0 + 20, y), '状態: ゴール待機中', font=self.font_m, fill=(180, 185, 190))
        y += 50
        d.text((ix0 + 20, y), f'経過 {time.time() - self.t0:5.1f} 秒',
               font=self.font_s, fill=(160, 168, 178))
        y += 36
        d.text((ix0 + 20, y), '凡例: ■赤=現在位置  ─青=計画経路  ─緑=走行軌跡  ★黄=目標',
               font=self.font_s, fill=(160, 168, 178))

        self.writer.write(np.asarray(im)[:, :, ::-1])
        self.frames += 1

    def close(self):
        self.writer.release()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/nav_demo.mp4'
    rclpy.init()
    n = NavRec(out)
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        print(f'[recorder_nav] {n.frames} フレーム({n.frames / FPS:.1f} 秒)を書きました: {out}',
              flush=True)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
