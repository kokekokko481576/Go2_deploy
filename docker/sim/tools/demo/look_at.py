#!/usr/bin/env python3
"""カメラ位置 (x,y,z) と注視点 (tx,ty,tz) から gz の姿勢（クォータニオン）を出す。
   python3 look_at.py x y z tx ty tz  → 'x y z qx qy qz qw'"""
import math, sys
x, y, z, tx, ty, tz = map(float, sys.argv[1:7])
dx, dy, dz = tx - x, ty - y, tz - z
yaw = math.atan2(dy, dx)
pitch = math.atan2(-dz, math.hypot(dx, dy))   # 下向きが正（gz のカメラは +X を見る）
cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
# roll=0: q = qz(yaw) * qy(pitch)
qw, qx, qy, qz = cy * cp, -sy * sp, cy * sp, sy * cp
print(f'{x} {y} {z} {qx:.5f} {qy:.5f} {qz:.5f} {qw:.5f}')
