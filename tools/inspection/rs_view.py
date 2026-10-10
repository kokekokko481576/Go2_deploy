#!/usr/bin/env python3
"""rs_snap の出力（_color.ppm / _depth.pgm）を見られる PNG にする（2026-09-24）。

  python3 rs_view.py ~/marker_detection/logs/rs/snap2
    → snap2_color.png（カラー）と snap2_both.png（カラーと深度を横に並べたもの）

深度は 0.2m(赤)〜2.0m(青) で色付けし、測れなかった画素は黒。
中央の白い十字が rs_snap の「中央21x21の中央値」を取った位置。
"""
import sys

import cv2
import numpy as np

prefix = sys.argv[1]
# rs_snap は PPM を RGB の並びで書くので、OpenCV の BGR に直す
color = cv2.cvtColor(cv2.imread(prefix + '_color.ppm'), cv2.COLOR_RGB2BGR)
depth = cv2.imread(prefix + '_depth.pgm', cv2.IMREAD_UNCHANGED).astype(np.float32)
cv2.imwrite(prefix + '_color.png', color)

n = np.clip((depth - 200) / (2000 - 200), 0, 1)
vis = cv2.applyColorMap((255 * (1 - n)).astype(np.uint8), cv2.COLORMAP_JET)
vis[depth == 0] = 0
h, w = depth.shape
for img in (color := color.copy(), vis):
    cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 40, 2)
cv2.putText(vis, 'depth 0.2m(red)-2.0m(blue), black=invalid', (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
cv2.imwrite(prefix + '_both.png', cv2.resize(np.hstack([color, vis]), (1280, 360)))
print(prefix + '_both.png')
