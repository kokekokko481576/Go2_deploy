#!/usr/bin/env python3
"""基部の到達誤差をアームで吸収できるかを幾何で解く（2026-09-23）。**実機もsimも要らない。**

## なぜ要るか

実機実測で、接近制御の最小動作単位が許容値より大きいと分かった（一歩0.12m・一回転18度に対し
`pos_tolerance`=0.06m・`turn_tolerance`=5度）。基部を細かく追い込む設計が成立しないので、
**基部は粗く寄せ、残差はアームで吸収する**しかない。その可否をここで判定する。

## 合格条件（2026-09-23にユーザー了承の仮値。検査要件が出たら置き換えること）

  ビード長 80mm を1枚に収める / 分解能 0.3mm/px / 視線角は面法線から±20度以内

撮影距離に翻訳すると、**上限は分解能、下限はカメラの最小測距**で決まる。

## 運動学

`d1_550_description/urdf/d1_arm.xacro` と `go2_description/xacro/robot.xacro` から起こした。
**j4・j6（ロール）を0に固定すると、残る回転(j1のヨー、j2/j3/j5のピッチ)は
`Rz(-j1)·Ry(j2+j3+j5)` にまとまる**ので、閉じた形で書ける:

    C    = base + Rz(-j1)·[ t2 + Ry(b)·t3 + Ry(b+c)·(t4+t5) + Ry(b+c+e)·(t6+cam) ]
    look = Rz(-j1)·Ry(b+c+e)·x̂        （b=j2, c=j3, e=j5）

先端ピッチが j2+j3+j5 の和になるのは #64 の記述どおり。総当たりが重かったのでこの形にした。

**マウント原点 (-0.05, 0, 0.06) とカメラ原点 (0.02, 0, 0.07) はどちらも実機未計測の仮値。**
実測値が出たら差し替えること（Go2_deploy #65）。
"""
import math

import numpy as np

# trunk → アーム基部（底面の中心）。**2026-09-23に実機で実測した値。**
#   測り方: 前脚の股関節軸→アーム底面中心の前後距離 A=215mm、
#           前後の股関節軸間 L=400mm（URDF値 386.8mm、目視の誤差範囲で一致）、
#           床→股関節軸 H1=125mm、床→アーム底面 H2=170mm。
#   前後 = 193.4(URDFの半分) - 215 = -21.6mm / 高さ = 170 - 125 = +45mm
#   底面は水平（rpy=0）、j1=0 でアームは機体前方を向く（simの規約どおり）。
#   前後は実測Lを採ると-15mmになる。目視誤差±7mm程度。
# 仮値は (-0.05, 0.0, 0.06) だった。実機は28mm前寄り・15mm低い。
MOUNT = np.array([-0.022, 0.0, 0.045])
CAM = np.array([0.02, 0.0, 0.07])        # link6 → カメラ。**実機未計測の仮値**
T1 = np.array([0.0, 0.0, 0.0738])
T2 = np.array([0.0, -0.0276, 0.0578])
T3 = np.array([0.0, -0.0004, 0.27])
T4 = np.array([0.05, 0.0275, 0.041325])
T5 = np.array([0.15468, -0.0258, 0.0001])
T6 = np.array([0.0777, 0.025822, -0.0010718])
BASE = MOUNT + T1
LIM_PITCH = 1.57      # j2/j3/j5
LIM_YAW = 2.36        # j1


def ry(th, v):
    """Ry(th) を定ベクトル v に掛ける。th は配列でよい。"""
    c, s = np.cos(th), np.sin(th)
    return np.stack([c * v[0] + s * v[2],
                     np.broadcast_to(v[1], c.shape),
                     -s * v[0] + c * v[2]], axis=-1)


def arm_local(b, c, e):
    """j1 を掛ける前の、基部から見たカメラ位置と光軸。"""
    v = (T2 + ry(b, T3) + ry(b + c, T4 + T5) + ry(b + c + e, T6 + CAM))
    look = ry(b + c + e, np.array([1.0, 0.0, 0.0]))
    return v, look


def rz(a, v):
    """Rz(-a) を掛ける（joint1 の軸は (0,0,-1)）。"""
    ca, sa = np.cos(-a), np.sin(-a)
    return np.stack([ca * v[..., 0] - sa * v[..., 1],
                     sa * v[..., 0] + ca * v[..., 1],
                     v[..., 2]], axis=-1)


def check_closed_form():
    """閉じた形が、関節を順に掛けていく素朴な計算と一致することを確かめる。"""
    def naive(q):
        chain = [(T1, (0, 0, -1)), (T2, (0, 1, 0)), (T3, (0, 1, 0)),
                 (T4, (1, 0, 0)), (T5, (0, 1, 0)), (T6, (1, 0, 0))]
        p, R = MOUNT.copy(), np.eye(3)
        for (t, axis), th in zip(chain, q):
            p = p + R @ t
            x, y, z = axis
            cs, sn, C = math.cos(th), math.sin(th), 1 - math.cos(th)
            R = R @ np.array([
                [x*x*C + cs,  x*y*C - z*sn, x*z*C + y*sn],
                [y*x*C+z*sn,  y*y*C + cs,   y*z*C - x*sn],
                [z*x*C-y*sn,  z*y*C + x*sn, z*z*C + cs]])
        return p + R @ CAM, R @ np.array([1.0, 0, 0])

    worst = 0.0
    for q in ([1.57, 1.2, 0, 0, 0, 0], [-0.8, 0.3, -0.9, 0, 0.5, 0], [0.2, -1.1, 1.3, 0, -0.7, 0]):
        a = np.array([q[0]])
        v, lk = arm_local(np.array([q[1]]), np.array([q[2]]), np.array([q[4]]))
        C = BASE + rz(a, v)[0]
        L = rz(a, lk)[0]
        Cn, Ln = naive(q)
        worst = max(worst, np.abs(C - Cn).max(), np.abs(L - Ln).max())
    return worst


# ---- 合格条件 → 撮影距離の窓 ----
BEAD_LEN = 0.080
RES_MM_PX = 0.3
VIEW_LIMIT = math.radians(20.0)
FRAME_MARGIN = 1.25
AIM_LIMIT = math.radians(10.0)

CAMERAS = {
    'D405':       (87.0, 1280, 0.07, '最小測距が短い'),
    'D435 深度':  (87.0, 1280, 0.30, '深度の実用下限0.3m'),
    'D435 RGB':   (69.0, 1920, 0.10, 'RGBのみ。深度は別'),
}


def window(fov_deg, px, min_range):
    k = 2 * math.tan(math.radians(fov_deg) / 2)
    return max(min_range, BEAD_LEN * FRAME_MARGIN / k), (RES_MM_PX / 1000.0) * px / k


FLOOR_Z = -0.32
BEAD_NOMINAL = np.array([0.0, -0.50, FLOOR_Z + 0.08])
BEAD_NORMAL = np.array([0.0, 0.0, 1.0])


def bead_in_trunk(dx, dy, dyaw):
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return Rz @ (BEAD_NOMINAL - np.array([dx, dy, 0.0])), Rz @ BEAD_NORMAL


class Grid:
    """関節グリッドを1回だけ作って使い回す。"""

    def __init__(self, axes, n_pitch=41, n_yaw=73):
        pit = np.linspace(-LIM_PITCH, LIM_PITCH, n_pitch)
        zero = np.array([0.0])
        b = pit
        c = pit if 2 in axes else zero
        e = pit if 4 in axes else zero
        B, C_, E = np.meshgrid(b, c, e, indexing='ij')
        v, look = arm_local(B.ravel(), C_.ravel(), E.ravel())
        self.a = np.linspace(-LIM_YAW, LIM_YAW, n_yaw)
        self.v, self.look = v, look
        self.n = v.shape[0]

    def feasible(self, bead, normal, d_min, d_max):
        """どれか1つでも合格条件を満たす関節姿勢があるか。"""
        b0 = bead - BASE
        for a in self.a:
            ca, sa = math.cos(a), math.sin(a)
            # Rz(-a) を掛ける代わりに、ビード側を Rz(a) で回して比べる
            bx = ca * b0[0] - sa * b0[1]
            by = sa * b0[0] + ca * b0[1]
            nx = ca * normal[0] - sa * normal[1]
            ny = sa * normal[0] + ca * normal[1]
            w = np.array([bx, by, b0[2]]) - self.v          # カメラ→ビード
            d = np.linalg.norm(w, axis=-1)
            m = (d >= d_min) & (d <= d_max)
            if not m.any():
                continue
            # カメラが床より下に潜っていないか（j1 で回してもzは変わらない）
            m &= (BASE[2] + self.v[:, 2]) > FLOOR_Z + 0.02
            if not m.any():
                continue
            u = w[m] / d[m][:, None]
            aim = np.einsum('ij,ij->i', self.look[m], u)
            m2 = aim >= math.cos(AIM_LIMIT)
            if not m2.any():
                continue
            nvec = np.array([nx, ny, normal[2]])
            view = -u[m2] @ nvec
            if (view >= math.cos(VIEW_LIMIT)).any():
                return True
        return False


def envelope(g, lo, hi, zs, xs, ys):
    """ビードを置ける範囲を掃く。返すのは合格した (x, y, z) の集合。"""
    ok = set()
    for z in zs:
        for x in xs:
            for y in ys:
                bead = np.array([x, y, z])
                if g.feasible(bead, BEAD_NORMAL, lo, hi):
                    ok.add((round(x, 3), round(y, 3), round(z, 3)))
    return ok


def main():
    err = check_closed_form()
    print(f'閉じた形と素朴な順運動学の差: {err:.2e} m（一致）\n', flush=True)
    print('=' * 74)
    print('合格条件(仮値): ビード長{:.0f}mm / {:.1f}mm/px / 視線角±{:.0f}度'
          .format(BEAD_LEN * 1000, RES_MM_PX, math.degrees(VIEW_LIMIT)))
    print('マウント{} / カメラ{} は**実機未計測の仮値**'.format(tuple(MOUNT), tuple(CAM)))
    print('=' * 74)
    print('\n【1】合格条件から決まる撮影距離の窓\n', flush=True)
    wins = {}
    for name, (fov, px, mn, note) in CAMERAS.items():
        lo, hi = window(fov, px, mn)
        wins[name] = (lo, hi)
        print(f'  {name:<11}画角{fov:>4.0f}度 {px:>5}px   下限{lo:>7.3f}m  上限{hi:>7.3f}m   '
              + ('窓あり' if lo < hi else '**窓なし。この条件では使えない**') + f'   ({note})',
              flush=True)

    # ビードを置ける範囲。床からの高さも振る（対象の高さは未確定なので）
    zs = [FLOOR_Z + h for h in (0.00, 0.08, 0.20, 0.35, 0.50)]
    xs = np.round(np.arange(-0.30, 0.3001, 0.05), 3)
    ys = np.round(np.arange(-0.70, -0.0499, 0.025), 3)

    print('\n【2】ビードを置ける範囲（＝アームが合格条件で撮れる範囲）\n', flush=True)
    print('  「横」は機体中心からビードまでの横方向距離[m]、「前後」は機体前後方向[m]', flush=True)
    for name, (lo, hi) in wins.items():
        if lo >= hi:
            continue
        print(f'\n  === {name}   撮影距離 {lo:.3f}〜{hi:.3f}m ===', flush=True)
        for label, axes in (('j1+j2 (2軸)', (0, 1)),
                            ('j1+j2+j3 (3軸)', (0, 1, 2)),
                            ('j1+j2+j3+j5 (4軸)', (0, 1, 2, 4))):
            g = Grid(axes)
            ok = envelope(g, lo, hi, zs, xs, ys)
            if not ok:
                print(f'    {label:<18} 合格域なし', flush=True)
                continue
            print(f'    {label:<18} 合格 {len(ok)} 点', flush=True)
            for z in zs:
                pts = [(x, y) for (x, y, zz) in ok if abs(zz - round(z, 3)) < 1e-6]
                if not pts:
                    print(f'        床から{z - FLOOR_Z:.2f}m: なし', flush=True)
                    continue
                yy = [-y for _x, y in pts]
                xx = [x for x, _y in pts]
                # その高さでの、前後方向に一番広く取れる横位置を探す
                best_y, best_w = None, -1
                for yv in sorted(set(yy)):
                    col = sorted(x for x, y in pts if abs(-y - yv) < 1e-6)
                    w = max(col) - min(col)
                    if w > best_w:
                        best_y, best_w, best_col = yv, w, col
                print(f'        床から{z - FLOOR_Z:.2f}m: 横 {min(yy):.3f}〜{max(yy):.3f}m / '
                      f'前後 {min(xx):+.2f}〜{max(xx):+.2f}m  '
                      f'（横{best_y:.3f}m のとき前後が最も広く {best_w:.2f}m）', flush=True)


if __name__ == '__main__':
    main()
