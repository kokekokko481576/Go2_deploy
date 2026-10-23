#!/usr/bin/env python3
"""撮影の代役のうち、**深度画像を合成する**もの（sim の通し試験専用。2026-10-07）。

アームの代役（d1_fake.py）の今の角度からカメラの位置と向きを出し、逆T字の部材（ベース板＋縦板）と
床を光線で当てて、rs_snap.cpp と同じ書式の <名前>_depth.pgm（mm・ビッグエンディアン）・
_color.png（深度の濃淡）・_info.txt を書く。aim_correct.py の閉ループ試験に使う。

部材の置き方（伏せた機体の trunk 座標。環境変数で変える）:
  SIM_ROOT_Y        縦板の根元の左右位置[m]（既定: 基準の撮影姿勢の光軸がベース板に当たる位置）
  SIM_ROOT_DY       上に足すずれ[m]（機体の止まる位置のばらつき。正=機体が部材から遠い）
  SIM_ROOT_YAW_DEG  部材の向きのずれ[度]
  SIM_PLATE_DX      縦板の区間の前後のずれ[m]（区間の長さ 0.34m）
  SIM_CAM_PITCH_ERR_DEG  **本物のカメラが、補正側のモデルより上を向いている角度**（モデルの誤差の再現）
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import aim_correct as ac  # noqa: E402

W, H = 1280, 720
FX, FY, CX, CY = 656.47, 655.74, 639.68, 370.20     # 10/1 の D405 実機の値
FLOOR = -0.125          # 伏せ
PLATE_T = 0.013         # ベース板の厚さ
WALL_H = 0.10           # 縦板の高さ
NEAR, FAR = 0.14, 0.16  # 根元からベース板の手前の縁・奥の縁まで
LEN = 0.34              # 縦板の区間
REF = [78.2, 54.3, 60.3, 0, -69.5, 0]


def nominal_root():
    """基準の撮影姿勢の光軸がベース板の上面に当たる点（x, y）。"""
    pos, R = ac.camera_frame(REF, 0.0)
    look = R[:, 2]
    t = (FLOOR + PLATE_T - pos[2]) / look[2]
    hit = pos + t * look
    return float(hit[0]), float(hit[1])


def render(q6, out_prefix):
    x0, y0 = nominal_root()
    yr = float(os.environ.get('SIM_ROOT_Y', y0)) - float(os.environ.get('SIM_ROOT_DY', 0.0))
    yaw = math.radians(float(os.environ.get('SIM_ROOT_YAW_DEG', 0.0)))
    xc = x0 + float(os.environ.get('SIM_PLATE_DX', 0.0))
    perr = float(os.environ.get('SIM_CAM_PITCH_ERR_DEG', 0.0))
    pos, R = ac.camera_frame(q6, perr)          # 本物のカメラ（モデルより perr 度上向き）

    v, u = np.mgrid[0:H, 0:W]
    dirs_c = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, dtype=float)], -1)
    dirs = dirs_c @ R.T                          # trunk 座標の光線（z成分はカメラの奥行き1に対応）
    # 部材の座標へ（根元の点 (xc, yr) を中心に -yaw 回す）
    c, s = math.cos(-yaw), math.sin(-yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    o = rot @ (pos - np.array([xc, yr, 0.0]))
    dv = dirs @ rot.T
    t_best = np.full((H, W), np.inf)

    def hit_plane(axis, value, lims):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (value - o[axis]) / dv[..., axis]
        p = o + t[..., None] * dv
        ok = t > 0
        for ax, lo, hi in lims:
            ok &= (p[..., ax] >= lo) & (p[..., ax] <= hi)
        np.minimum(t_best, np.where(ok, t, np.inf), out=t_best)

    top = FLOOR + PLATE_T
    hit_plane(2, FLOOR, [])                                                  # 床
    hit_plane(2, top, [(0, -0.35, 0.35), (1, -FAR, NEAR)])                   # ベース板の上面
    hit_plane(1, 0.0, [(0, -LEN / 2, LEN / 2), (2, top, top + WALL_H)])      # 縦板の機体側の面
    hit_plane(2, top + WALL_H, [(0, -LEN / 2, LEN / 2), (1, -0.01, 0.0)])    # 縦板の上端
    depth = np.where(np.isfinite(t_best), t_best, 0.0)                       # 奥行き(=t、dirs_c の z=1)
    depth[(depth < 0.07) | (depth > 0.5)] = 0.0                              # D405 の測距範囲の外は抜ける

    mm = np.clip(depth * 1000 + 0.5, 0, 65535).astype('>u2')
    Path(out_prefix + '_depth.pgm').write_bytes(f'P5\n{W} {H}\n65535\n'.encode() + mm.tobytes())
    Path(out_prefix + '_info.txt').write_text(
        f'color {W}x{H}  fx={FX} fy={FY} cx={CX} cy={CY}\n'
        f'depth_scale=0.000100 m/unit\n(sim render) q={q6} root_y={yr:.3f} yaw={math.degrees(yaw):.1f} cam_pitch_err={perr}\n')
    try:
        from PIL import Image
        g = np.where(depth > 0, 255 - np.clip((depth - 0.1) / 0.4 * 200, 0, 200), 0).astype(np.uint8)
        Image.fromarray(g).save(out_prefix + '_color.png')
    except ImportError:
        Path(out_prefix + '_color.png').write_text('sim render (PIL なし)')


def main():
    name = sys.argv[1]
    # inspect_run.py・rs_snap_fake.sh と同じ保存先（RS_OUT）
    out = Path(os.environ.get('RS_OUT', str(Path.home() / 'marker_detection' / 'logs' / 'rs')))
    out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(HERE))
    import d1_fake
    q = d1_fake.now_angles(d1_fake.load())[:6]
    render(q, str(out / name))
    print(f'保存: {out / name}_color.png (sim render, q={" ".join(f"{v:.1f}" for v in q)})')


if __name__ == '__main__':
    main()
