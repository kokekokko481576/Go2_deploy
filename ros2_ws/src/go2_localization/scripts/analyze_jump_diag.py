#!/usr/bin/env python3
"""jump_diagバッグ(#54切り分け用)の解析。particle_cloudの広がり・amcl_poseの跳躍・
gz_ground_truth/poseの静止区間を突き合わせ、「飛ぶ直前にパーティクル雲が広がるか」を見る。
"""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else 'jump_diag'

storage_options = rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

amcl = []       # (t, x, y)
gt = []         # (t, x, y)
pc = []         # (t, n, spread_std)

while reader.has_next():
    topic, data, t_ns = reader.read_next()
    t = t_ns / 1e9
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)
    if topic == '/go2_localization/amcl_pose':
        amcl.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
    elif topic == '/gz_ground_truth/pose':
        gt.append((t, msg.pose.position.x, msg.pose.position.y))
    elif topic == '/particle_cloud':
        n = len(msg.particles)
        if n > 0:
            xs = [p.pose.position.x for p in msg.particles]
            ys = [p.pose.position.y for p in msg.particles]
            mx, my = sum(xs) / n, sum(ys) / n
            spread = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n)
        else:
            spread = 0.0
        pc.append((t, n, spread))

print(f'amcl_pose: {len(amcl)}件, particle_cloud: {len(pc)}件, ground_truth: {len(gt)}件')

# amcl_poseの連続サンプル間の移動距離(=跳躍量)を計算
print('\n=== amcl_poseの跳躍イベント(前サンプルから0.3m以上動いた瞬間) ===')
jumps = []
for i in range(1, len(amcl)):
    t0, x0, y0 = amcl[i - 1]
    t1, x1, y1 = amcl[i]
    d = math.hypot(x1 - x0, y1 - y0)
    if d > 0.3:
        jumps.append((t1, d, x0, y0, x1, y1))

for t1, d, x0, y0, x1, y1 in jumps:
    # 直近0.5秒以内のparticle_cloudの広がりを探す
    nearby_pc = [p for p in pc if abs(p[0] - t1) < 0.5]
    spread_note = f'spread={nearby_pc[-1][2]:.3f} n={nearby_pc[-1][1]}' if nearby_pc else 'particle_cloud無し'
    print(f't={t1:.1f} jump={d:.2f}m  ({x0:.2f},{y0:.2f})->({x1:.2f},{y1:.2f})  直近pc: {spread_note}')

print(f'\n跳躍イベント数: {len(jumps)}')

# 通常時(跳躍が無い区間)のparticle_cloud spread平均 と 跳躍直前のspread平均を比較
jump_times = set(round(j[0], 1) for j in jumps)
near_jump_spreads = []
normal_spreads = []
for t, n, spread in pc:
    is_near_jump = any(abs(t - jt) < 0.5 for jt in jump_times)
    (near_jump_spreads if is_near_jump else normal_spreads).append(spread)

def avg(xs):
    return sum(xs) / len(xs) if xs else float('nan')

print(f'\n通常時のparticle spread平均: {avg(normal_spreads):.4f} (n={len(normal_spreads)})')
print(f'跳躍直前後0.5秒のparticle spread平均: {avg(near_jump_spreads):.4f} (n={len(near_jump_spreads)})')

# ground truthが静止している区間(移動0.02m未満)を特定し、その間のamcl跳躍を数える
print('\n=== ground truth静止区間でのamcl_pose挙動 ===')
static_windows = []
if gt:
    seg_start = gt[0]
    for i in range(1, len(gt)):
        t0, x0, y0 = gt[i - 1]
        t1, x1, y1 = gt[i]
        if math.hypot(x1 - x0, y1 - y0) > 0.02:
            if t0 - seg_start[0] > 5.0:
                static_windows.append((seg_start[0], t0, seg_start[1], seg_start[2]))
            seg_start = gt[i]
    if gt[-1][0] - seg_start[0] > 5.0:
        static_windows.append((seg_start[0], gt[-1][0], seg_start[1], seg_start[2]))

for w_start, w_end, gx, gy in static_windows:
    dur = w_end - w_start
    amcl_in_window = [(t, x, y) for t, x, y in amcl if w_start <= t <= w_end]
    if len(amcl_in_window) < 2:
        continue
    xs = [p[1] for p in amcl_in_window]
    ys = [p[2] for p in amcl_in_window]
    amcl_range = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    print(f'静止区間 {dur:.1f}秒 (真値=({gx:.2f},{gy:.2f})): '
          f'amcl_poseサンプル数={len(amcl_in_window)}, 動いた範囲={amcl_range:.2f}m, '
          f'x範囲=[{min(xs):.2f},{max(xs):.2f}] y範囲=[{min(ys):.2f},{max(ys):.2f}]')
