#!/usr/bin/env python3
"""コンテナ内で、指定トピックの画像を1枚 PNG に保存する（構図の確認用）。
   python3 grab_image.py /overview_cam /tmp/overview.png"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2

topic, out = sys.argv[1], sys.argv[2]
rclpy.init()
n = Node('grab_image')
got = {}
def cb(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    if m.encoding in ('rgb8',):
        a = a[:, :, ::-1]
    got['img'] = a
n.create_subscription(Image, topic, cb, 1)
import time
t0 = time.time()
while 'img' not in got and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'img' in got:
    cv2.imwrite(out, got['img']); print('saved', out, got['img'].shape)
else:
    print('no image from', topic)
