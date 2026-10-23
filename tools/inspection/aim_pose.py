#!/usr/bin/env python3
"""立った機体から、アーム先端のカメラを床上の1点へ向ける関節角を出す（2026-09-28）。

**実機もsimも要らない。** 運動学は arm_reach_study.py と同じもの（実機で検証済み:
姿勢(90,20,0,...)の先端位置が予測と2〜6mmで一致）を使う。

j1〜j3（SDKの angle0〜2）は符号とゼロ点を実機で確定済みなので、出た値をそのまま渡せる。
**j5（angle4、手首ピッチ）は符号もゼロ点も未確認**なので、既定では0に固定して3軸で解く。

仮定（実機で確かめること）:
  - 立った機体の床は trunk原点の 0.32m 下（arm_reach_study.FLOOR_Z）
  - カメラは link6 から (0.02, 0, 0.07) で、光軸は link6 の +x（**ブラケット未製作の仮値**）
  - 胴体と頭は箱で近似（下の BODY_BOXES）。脚は胴体下の箱

使い方:
  python3 tools/aim_pose.py                 # 部材に対する置き方 A/B の両方を出す
  python3 tools/aim_pose.py --dist 0.30     # カメラ→対象の距離
"""
import argparse
import math

import numpy as np
from scipy.optimize import minimize

import arm_reach_study as ars
from arm_reach_study import BASE, T2, T3, T4, T5, T6, LIM_PITCH, LIM_YAW, ry, rz

# link6 → カメラのレンズ。**2026-09-28 ユーザー申告の取り付け。**
# グリッパでカメラの支柱を横から挟む。レンズはグリッパ先端（link6 +0.13m）から
# 上に65mm、アーム側から見て右に10mm。光軸は link6 の +x（先端の向き）と仮定。
# グリッパ先端 0.13m は実機測定の値（2026-09-23）。挟む位置が先端より奥なら x を減らす
CAM = np.array([0.13, -0.010, 0.065])
FLOOR_Z = ars.FLOOR_Z          # 立ち: trunk原点の0.32m下。伏せ: 0.125m下（--lying）

# trunk座標系の障害物の箱 (xmin, xmax, ymin, ymax, zmin, zmax)[m]。
# 胴体 0.376x0.094x0.114（URDFの衝突形状）に、頭・股関節・背面のアーム台座ぶん余裕を足した
def body_boxes():
    return [
        (-0.30, 0.37, -0.10, 0.10, -0.07, 0.09),     # 胴体・頭
        (-0.30, 0.30, -0.20, 0.20, FLOOR_Z, -0.07),   # 脚の占める空間
    ]
CLEARANCE = 0.04      # 箱・床から離す距離[m]


def chain_points(q):
    """関節の位置を順に返す（最後がカメラ）。q=(j1,j2,j3,j5)。"""
    a, b, c, e = q
    pts_local = [np.zeros(3), T2.copy()]
    p = T2 + ry(np.array(b), T3)
    pts_local.append(p)
    p = p + ry(np.array(b + c), T4)
    pts_local.append(p)
    p = p + ry(np.array(b + c), T5)
    pts_local.append(p)
    p = p + ry(np.array(b + c + e), T6)
    pts_local.append(p)
    p = p + ry(np.array(b + c + e), CAM)
    pts_local.append(p)
    pts = [BASE + rz(np.array(a), np.asarray(v, dtype=float)) for v in pts_local]
    look = rz(np.array(a), ry(np.array(b + c + e), np.array([1.0, 0.0, 0.0])))
    return pts, look


def box_penetration(p, box, margin):
    """点 p が箱（margin だけ膨らませた）にどれだけ入り込んでいるか[m]。外なら0。"""
    x0, x1, y0, y1, z0, z1 = box
    d = min(p[0] - (x0 - margin), (x1 + margin) - p[0],
            p[1] - (y0 - margin), (y1 + margin) - p[1],
            p[2] - (z0 - margin), (z1 + margin) - p[2])
    return max(0.0, d)


def collision(q):
    """リンクの線分上の点が箱・床に入り込んだ量の和。基部付近（最初の線分）は除く。"""
    pts, _ = chain_points(q)
    pen = 0.0
    for i in range(2, len(pts)):
        for s in np.linspace(0.0, 1.0, 6):
            p = pts[i - 1] * (1 - s) + pts[i] * s
            for box in body_boxes():
                pen += box_penetration(p, box, CLEARANCE)
            pen += max(0.0, FLOOR_Z + CLEARANCE - p[2])
    return pen


def cost(q, target, dist, view_max):
    pts, look = chain_points(q)
    cam = pts[-1]
    w = target - cam
    d = np.linalg.norm(w)
    u = w / d
    aim = math.acos(np.clip(np.dot(look, u), -1, 1))           # 光軸と対象のずれ
    view = math.acos(np.clip(-u[2], -1, 1))                     # 真上からの傾き
    return (50 * aim ** 2 + 20 * (d - dist) ** 2
            + 5 * max(0.0, view - view_max) ** 2
            + 2000 * collision(q) ** 2)


def solve(target, dist, view_max, use_j5=False):
    """多点から最適化して、条件を満たす中で最も良い解を返す。"""
    best = None
    bounds = [(-LIM_YAW, LIM_YAW), (-LIM_PITCH, LIM_PITCH), (-LIM_PITCH, LIM_PITCH),
              (-LIM_PITCH, LIM_PITCH) if use_j5 else (0.0, 0.0)]
    yaw0 = -math.atan2(target[1] - BASE[1], target[0] - BASE[0])   # j1は正で右
    for b0 in np.linspace(-0.5, 1.4, 6):
        for c0 in np.linspace(-1.2, 1.4, 6):
            x0 = [yaw0, b0, c0, 0.0]
            r = minimize(cost, x0, args=(target, dist, view_max), bounds=bounds,
                         method='L-BFGS-B')
            if best is None or r.fun < best.fun:
                best = r
    return best.x


def report(name, target, q, dist):
    pts, look = chain_points(q)
    cam = pts[-1]
    w = target - cam
    d = np.linalg.norm(w)
    u = w / d
    aim = math.degrees(math.acos(np.clip(np.dot(look, u), -1, 1)))
    view = math.degrees(math.acos(np.clip(-u[2], -1, 1)))
    col = collision(q)
    deg = [math.degrees(v) for v in q]
    print(f'\n[{name}] 対象 trunk座標 ({target[0]:+.3f}, {target[1]:+.3f}, 床+{target[2] - FLOOR_Z:.3f})')
    print(f'  SDK: set_joints {deg[0]:.1f} {deg[1]:.1f} {deg[2]:.1f} 0 {deg[3]:.1f} 0 <angle6は現在値>')
    print(f'  カメラ位置 ({cam[0]:+.3f}, {cam[1]:+.3f}, 床+{cam[2] - FLOOR_Z:.3f})  '
          f'対象まで {d:.3f}m（目標{dist:.2f}）  光軸のずれ {aim:.1f}度  真上からの傾き {view:.1f}度')
    low = min(p[2] for p in pts) - FLOOR_Z
    print(f'  アームの最低点 床+{low:.3f}m  干渉 {"なし" if col < 1e-4 else f"あり({col:.3f})"}')
    ok = aim < 3 and abs(d - dist) < 0.03 and col < 1e-4
    print('  → ' + ('使える' if ok else '**条件を満たせない**（上の値を見て判断すること）'))


def main():
    global FLOOR_Z
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=0.30,
                    help='レンズ→対象の距離[m]。340mmを1枚に収めるならD435i RGB(69度)で約0.30m')
    ap.add_argument('--view-max', type=float, default=20.0, help='真上からの傾きの上限[度]')
    ap.add_argument('--lying', action='store_true', help='伏せた機体（床= trunk原点の0.125m下）')
    ap.add_argument('--x', type=float, default=None, help='対象の前後位置[m]（trunk座標）')
    ap.add_argument('--y', type=float, default=None, help='対象の左右位置[m]（左が正）')
    ap.add_argument('--j5', action='store_true', help='手首ピッチ(angle4)も使う（符号未確認）')
    a = ap.parse_args()
    FLOOR_Z = -0.125 if a.lying else ars.FLOOR_Z

    # 2026-09-28 の配置: 部材の手前の長辺(700mm)に右前足・右後ろ足を重ねて横付け。
    # 右前足の軸（前の股関節 x=+0.193、脚の横位置 y=-0.142）を部材の左前の角に合わせる。
    # ビードは手前の縁から140mm奥（右）、左端から後ろへ340mm。狙うのは区間の中央
    edge_y, corner_x = -0.142, 0.193
    x = corner_x - 0.340 / 2 if a.x is None else a.x
    y = edge_y - 0.140 if a.y is None else a.y
    z = FLOOR_Z + 0.013
    q = solve(np.array([x, y, z]), a.dist, math.radians(a.view_max), a.j5)
    report(('伏せ' if a.lying else '立ち') + ' 横付け', np.array([x, y, z]), q, a.dist)
    for name, xe in (('区間の前端', corner_x), ('区間の後端', corner_x - 0.340)):
        pts, look = chain_points(q)
        t = np.array([xe, y, z])
        u = (t - pts[-1]) / np.linalg.norm(t - pts[-1])
        print(f'  {name}(x={xe:+.3f}) は光軸から {math.degrees(math.acos(np.dot(look, u))):.1f}度')


if __name__ == '__main__':
    main()
