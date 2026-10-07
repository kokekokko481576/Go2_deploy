#!/usr/bin/env python3
"""撮った深度画像から隅肉の根元を見つけ、アームの姿勢を直す（2026-10-07）。**ROS 不要。**

機体の止まる位置は±7cmばらつく（2026-09-29）。決め打ちの撮影姿勢では根元が画面の端に寄ったり
外れたりする（10/1 の外れ: 根元はカメラから0.36m・光軸から左31度）。そこで

  1. 撮る（深度つき）
  2. 深度の点群から、ベース板の面と、それに垂直な縦板の面を当てはめる。2つの交線が隅肉の根元
  3. 根元の線上で「いまの光軸に一番近い点」を狙いにする（線に沿った位置は変えない。どこを撮るかは計画の仕事）
  4. その点をモデル（aim_pose、実機で2〜6mmの一致を確認済み）で機体座標へ移し、
     いまの姿勢の近くで「光軸がその点を向き、距離が dist」になる関節角を解き直す
  5. いまの姿勢から新しい姿勢へ、**各関節がどの順に着いても干渉しない**ことを確かめて動かす
     （merge_steps.box_worst。だめなら1関節ずつの順番を探す）

を1〜2回くり返す。見た目ではなく形で探すので、光り方や汚れに左右されにくい。
**モデルの誤差（カメラの取り付け角など）は、見えた点を目標に解き直すので近くでは打ち消し合う**
（9/28 に手で行った方法の自動化）。それでも残る分はくり返しで詰める。

使い方（手元の写真で試す）:
  python3 aim_correct.py ~/marker_detection/logs/rs/inspect_1001_132926_2_a4_-69.5 \\
      --q 78.2 54.3 60.3 0 -69.5 0 --lying

深度PGMは rs_snap.cpp が書く**換算済みのmm・ビッグエンディアン**。info の depth_scale は使わない。
"""
import argparse
import itertools
import math
import re
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

import aim_pose as ap
import merge_steps as ms
from arm_reach_study import ry, rz

# モデルのカメラ光軸に足す縦の傾き[度]（正で上向き）。D405 の取り付け角は未計測なので 0。
# 実機で系統的にずれるなら、ここを合わせるとくり返しの回数が減る（合っていなくても収束はする）
CAM_PITCH_DEG = 0.0


# ---------------------------------------------------------------- 読み込み
def load_snap(prefix):
    """(深度[m] の2次元配列, fx, fy, cx, cy)。"""
    prefix = str(prefix)
    info = Path(prefix + '_info.txt').read_text()
    fx, fy, cx, cy = [float(re.search(k + r'=([\d.]+)', info).group(1)) for k in ('fx', 'fy', 'cx', 'cy')]
    with open(prefix + '_depth.pgm', 'rb') as f:
        if f.readline().strip() != b'P5':
            raise ValueError('16bit PGM(P5) ではない')
        w, h = map(int, f.readline().split())
        f.readline()
        d = np.frombuffer(f.read(), dtype='>u2').reshape(h, w).astype(float) * 0.001
    return d, fx, fy, cx, cy


# ---------------------------------------------------------------- 検出
def _plane_ransac(P, rng, thr, iters, perp_to=None, chunk=64):
    """RANSAC で一番多くの点が乗る平面。候補をまとめて評価する（1つずつだと1枚3秒かかった）。"""
    if len(P) < 3:
        return None
    best_n, best_c, best_cnt = None, None, 0
    for k in range(0, iters, chunk):
        m = min(chunk, iters - k)
        idx = rng.integers(0, len(P), size=(m, 3))
        a, b, c = P[idx[:, 0]], P[idx[:, 1]], P[idx[:, 2]]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n, axis=1)
        good = nn > 1e-9
        if perp_to is not None:
            good &= np.abs((n / np.maximum(nn, 1e-12)[:, None]) @ perp_to) <= 0.34   # 70度未満は縦板ではない
        if not good.any():
            continue
        n = n[good] / nn[good][:, None]
        a = a[good]
        cnt = (np.abs(P @ n.T - np.sum(n * a, axis=1)) < thr).sum(0)
        g = int(np.argmax(cnt))
        if cnt[g] > best_cnt:
            best_n, best_c, best_cnt = n[g], a[g], int(cnt[g])
    if best_n is None or best_cnt < 3:
        return None
    inl = np.abs((P - best_c) @ best_n) < thr
    Q = P[inl]
    c0 = Q.mean(0)
    n = np.linalg.svd(Q - c0)[2][-1]
    return n, c0, np.abs((P - c0) @ n) < thr


def detect_root(depth, fx, fy, cx, cy, stride=6, thr=0.004, min_wall=300, seed=0,
                end_margin=0.02, border_frac=0.10, extend=0.10, min_wall_height=0.04, up_cam=None):
    """隅肉の根元の線をカメラ座標（x右・y下・z前、m）で返す。見つからなければ (None, 理由)。

    狙う点は、線上で光軸に一番近い点。ただし**縦板が画面の中で途切れていれば、そこが部材の端**なので
    端から end_margin だけ内側までに留める（10/1 の外れでは縦板が画面の中ほどで終わっていた）。
    線の端が画面の縁（幅・高さの border_frac 以内）に掛かっているときだけ、画面の外へ extend まで延ばしてよい。

    up_cam（カメラ座標での真上。モデルから出す）を渡すと、2つの面のうち水平に近いほうをベース板とする。
    渡さないと「一番大きい面＝ベース板」とみなすので、縦板が画面の大半を占めると取り違える。

    戻り値の dict: a, b（線分の両端）, target（線上で光軸に一番近い点）, off_deg, range, n_base, n_wall
    """
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    ok = (z > 0.05) & (z < 0.8)
    if ok.sum() < 500:
        return None, f'有効な深度が少ない（{ok.sum()}点）'
    P = np.stack([(u[ok] - cx) * z[ok] / fx, (v[ok] - cy) * z[ok] / fy, z[ok]], 1)
    rng = np.random.default_rng(seed)
    base = _plane_ransac(P, rng, thr, 300)
    if base is None:
        return None, '面が見つからない'
    n1, c1, in1 = base
    rest = P[~in1]
    wall = _plane_ransac(rest, rng, thr, 400, perp_to=n1)
    if wall is None or wall[2].sum() < min_wall:
        return None, f'縦板が見つからない（根元が写っていない。垂直な面の点 {0 if wall is None else wall[2].sum()}）'
    n2, c2, in2 = wall
    if up_cam is not None:
        if abs(n2 @ up_cam) > abs(n1 @ up_cam):
            # 大きいほうが縦板だった（機体が近い・カメラが上を向いている）。入れ替える
            in2_full = np.zeros(len(P), bool)
            in2_full[np.where(~in1)[0][in2]] = True
            n1, c1, n2, c2 = n2, c2, n1, c1
            in1, rest = in2_full, P[in1]
            in2 = np.ones(len(rest), bool)
        if abs(n1 @ up_cam) < math.cos(math.radians(30)):
            return None, 'ベース板らしい水平な面が見つからない'
    between = math.degrees(math.acos(min(1.0, abs(n1 @ n2))))
    if abs(between - 90.0) > 15.0:
        return None, f'縦板らしい面がベース板と垂直でない（{between:.0f}度）'
    W = rest[in2]
    # 縦板は約10cmの高さでベース板からカメラ側へ立ち上がる。ベース板の縁の側面（厚さ13mm）や、
    # 床から見たベース板の縁も「垂直な面」なので、高さと向きで除く（10/1 の3枚目で縁を拾った）
    up = n1 if (-c1) @ n1 > 0 else -n1          # ベース板からカメラ側を向く法線
    hgt = (W - c1) @ up
    h_lo, h_hi = np.percentile(hgt, [5, 95])
    if h_hi - h_lo < min_wall_height or np.median(hgt) < 0:
        return None, (f'垂直な面はあるが縦板ではない（高さ {100 * (h_hi - h_lo):.1f}cm・'
                      f'{"カメラ側" if np.median(hgt) >= 0 else "奥側"}。ベース板の縁の側面など）')
    d = np.cross(n1, n2)
    d /= np.linalg.norm(d)
    # 2平面の交線上の1点（2つの平面の式と、線の向きに垂直な条件の3式を解く）
    A = np.stack([n1, n2, d])
    p0 = np.linalg.solve(A, np.array([n1 @ c1, n2 @ c2, d @ p0_hint(c1, c2)]))
    t = (W - p0) @ d
    t0, t1 = np.percentile(t, [2, 98])
    a, b = p0 + t0 * d, p0 + t1 * d
    # 線上で光軸 (0,0,s) に一番近い点
    ez = np.array([0.0, 0.0, 1.0])
    m = np.array([[d @ d, -d @ ez], [d @ ez, -ez @ ez]])
    rhs = np.array([-(p0 @ d), -(p0 @ ez)])
    tt, _ = np.linalg.solve(m, rhs)

    def at_border(X):
        uu, vv = fx * X[0] / X[2] + cx, fy * X[1] / X[2] + cy
        return (uu < w * border_frac or uu > w * (1 - border_frac)
                or vv < h * border_frac or vv > h * (1 - border_frac))
    lo = t0 - extend if at_border(a) else t0 + end_margin
    hi = t1 + extend if at_border(b) else t1 - end_margin
    if lo > hi:      # 見えている縦板が短すぎる
        lo = hi = (t0 + t1) / 2
    tt = float(np.clip(tt, lo, hi))
    target = p0 + tt * d
    off = math.degrees(math.atan2(math.hypot(target[0], target[1]), target[2]))
    return dict(a=a, b=b, target=target, off_deg=off, range=float(np.linalg.norm(target)),
                n_base=int(in1.sum()), n_wall=int(in2.sum()), angle_between=between,
                a_border=at_border(a), b_border=at_border(b)), ''


def p0_hint(c1, c2):
    return (c1 + c2) / 2


# ---------------------------------------------------------------- モデル（カメラの姿勢）
def q_rad(q6):
    """SDK の angle0〜5[度] → モデルの (j1, j2, j3, j5)[rad]。angle3/5 は 0 前提。"""
    return [math.radians(q6[0]), math.radians(q6[1]), math.radians(q6[2]), math.radians(q6[4])]


def camera_frame(q6, pitch_deg=None):
    """カメラの位置と、光学座標の3軸（x右・y下・z前）を trunk 座標で返す。"""
    pitch = math.radians(CAM_PITCH_DEG if pitch_deg is None else pitch_deg)
    a, b, c, e = q_rad(q6)
    pts, _ = ap.chain_points([a, b, c, e])
    th = np.array(b + c + e - pitch)     # 上向きに傾ける = ピッチを減らす（正で先端が下がる）
    ex = np.asarray(rz(np.array(a), ry(th, np.array([1.0, 0.0, 0.0]))))      # 前（光軸）
    ey = np.asarray(rz(np.array(a), np.array([0.0, 1.0, 0.0])))              # 左
    ez = np.asarray(rz(np.array(a), ry(th, np.array([0.0, 0.0, 1.0]))))      # 上
    return pts[-1], np.stack([-ey, -ez, ex], 1)    # 列が光学座標の x, y, z


def cam_to_trunk(q6, p_cam, pitch_deg=None):
    pos, R = camera_frame(q6, pitch_deg)
    return pos + R @ p_cam


# ---------------------------------------------------------------- 解き直し
def solve_aim(q6, target, dist, ref_look, max_change_deg=30.0, pitch_deg=None):
    """いまの姿勢 q6 の近くで、光軸が target を向き距離が dist になる姿勢（angle0〜5[度]）を返す。"""
    x0 = np.array([q6[0], q6[1], q6[2], q6[4]], dtype=float)

    def pose(x):
        return [x[0], x[1], x[2], 0.0, x[3], 0.0]

    def cost(x):
        pos, R = camera_frame(pose(x), pitch_deg)
        look = R[:, 2]
        w = target - pos
        d = np.linalg.norm(w)
        aim = math.acos(np.clip(look @ (w / d), -1, 1))
        att = math.acos(np.clip(look @ ref_look, -1, 1))
        return (50 * aim ** 2 + 20 * (d - dist) ** 2 + 2 * att ** 2
                + 2000 * ms.penetration(pose(x)) ** 2
                + 1e-4 * float(np.sum((x - x0) ** 2)))

    lim = [(-135, 135), (-90, 90), (-90, 90), (-90, 90)]
    bounds = [(max(lo, v - max_change_deg), min(hi, v + max_change_deg)) for v, (lo, hi) in zip(x0, lim)]
    r = minimize(cost, x0, bounds=bounds, method='L-BFGS-B')
    return [round(float(v), 1) for v in pose(r.x)]


def aim_quality(q6, target, pitch_deg=None):
    pos, R = camera_frame(q6, pitch_deg)
    w = target - pos
    d = float(np.linalg.norm(w))
    return math.degrees(math.acos(np.clip(R[:, 2] @ (w / d), -1, 1))), d


def safe_moves(cur, new, single=False):
    """cur → new を安全に動かす指令の列。まとめて送れればその1つ、だめなら1関節ずつの順番を探す。

    single=True なら、まとめて送らず必ず1関節ずつにする（実機の多関節同時指令を確かめるまでの逃げ道）。
    どれも無理なら None。
    """
    if not single and ms.box_worst(cur, new) == 0.0:
        return [new]
    changed = [i for i in range(6) if abs(cur[i] - new[i]) > 1e-6]
    for order in itertools.permutations(changed):
        p = list(cur)
        steps = []
        ok = True
        for i in order:
            q = list(p)
            q[i] = new[i]
            if ms.box_worst(p, q) > 0.0:
                ok = False
                break
            steps.append(q)
            p = q
        if ok:
            return steps
    return None


def save_overlay(prefix, det, fx, fy, cx, cy):
    """見つけた根元の線（緑）と狙う点（赤）を写真に重ねて <接頭辞>_root.png に保存する。見間違いの確認用。"""
    try:
        from PIL import Image, ImageDraw
        im = Image.open(prefix + '_color.png').convert('RGB')
    except Exception:
        return None
    pr = lambda X: (fx * X[0] / X[2] + cx, fy * X[1] / X[2] + cy)   # noqa: E731
    d = ImageDraw.Draw(im)
    d.line([pr(det['a']), pr(det['b'])], fill=(0, 255, 0), width=6)
    t = pr(det['target'])
    d.ellipse([t[0] - 14, t[1] - 14, t[0] + 14, t[1] + 14], outline=(255, 0, 0), width=5)
    d.line([(cx - 20, cy), (cx + 20, cy)], fill=(255, 255, 255), width=3)
    d.line([(cx, cy - 20), (cx, cy + 20)], fill=(255, 255, 255), width=3)
    im.save(prefix + '_root.png')
    return prefix + '_root.png'


def plan_correction(prefix, q6, dist, ref_q6, lying=True, pitch_deg=None, single=False):
    """1枚の写真から、直すべきか・直すならどの指令列か を決める。

    戻り値 dict: found, reason, off_deg, range, new_q, moves
    """
    ap.FLOOR_Z = -0.125 if lying else ap.ars.FLOOR_Z
    depth, fx, fy, cx, cy = load_snap(prefix)
    _, Rq = camera_frame(q6, pitch_deg)
    det, why = detect_root(depth, fx, fy, cx, cy, up_cam=Rq.T @ np.array([0.0, 0.0, 1.0]))
    if det is None:
        return dict(found=False, reason=why)
    save_overlay(prefix, det, fx, fy, cx, cy)
    tgt = cam_to_trunk(q6, det['target'], pitch_deg)
    _, Rref = camera_frame(ref_q6, pitch_deg)
    new = solve_aim(q6, tgt, dist, Rref[:, 2], pitch_deg=pitch_deg)
    aim, d = aim_quality(new, tgt, pitch_deg)
    out = dict(found=True, reason='', off_deg=det['off_deg'], range=det['range'], target_trunk=tgt,
               new_q=new, model_aim_deg=aim, model_dist=d, det=det)
    if ms.penetration(new) > 0.0:
        out.update(moves=None, reason='解き直した姿勢がモデル上で干渉する')
        return out
    out['moves'] = safe_moves(list(q6), new, single)
    if out['moves'] is None:
        out['reason'] = 'いまの姿勢から干渉せずに動かす順番が無い'
    return out


def correct_loop(take, move, q6, ref_q6, dist=0.22, lying=True, max_iter=2,
                 ok_deg=3.0, ok_dist=0.03, max_from_ref=35.0, single=False, log=print):
    """撮る→見つける→直す を最大 max_iter 回。take(k) は撮った接頭辞を、move(q6) は動かしたことを返す。

    戻り値 (最後の姿勢, 合ったか)。見つからない・安全に動かせないときは、その場の姿勢のまま返す
    （呼び出し側はそのまま撮る）。基準姿勢から max_from_ref 度より大きく離れる直し方はしない
    （見間違いで大きく振り回さないため）。
    """
    q = list(q6)
    for k in range(max_iter + 1):
        prefix = take(k)
        r = plan_correction(prefix, q, dist, ref_q6, lying, single=single)
        if not r['found']:
            log(f'  [補正] 根元が見つからない: {r["reason"]}。この姿勢のまま撮る')
            return q, False
        log(f'  [補正 {k + 1}] 根元は光軸から {r["off_deg"]:.1f}度・{r["range"]:.3f}m（目標 {dist:.2f}m）')
        if r['off_deg'] <= ok_deg and abs(r['range'] - dist) <= ok_dist:
            log('  [補正] 合っている')
            return q, True
        if k == max_iter:
            log(f'  [補正] {max_iter}回直しても合わない。この姿勢のまま撮る')
            return q, False
        far = max(abs(r['new_q'][i] - ref_q6[i]) for i in range(6))
        if far > max_from_ref:
            log(f'  [補正] 基準姿勢から {far:.0f}度離れる直し方になる（上限 {max_from_ref:.0f}度）。直さずに撮る')
            return q, False
        if r.get('moves') is None:
            log(f'  [補正] {r["reason"]}。直さずに撮る')
            return q, False
        for m in r['moves']:
            move(m)
        q = list(r['new_q'])
    return q, False


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('prefix', help='撮影の接頭辞（<接頭辞>_depth.pgm と _info.txt がある）')
    p.add_argument('--q', type=float, nargs=6, required=True, help='撮ったときの angle0〜5[度]')
    p.add_argument('--ref', type=float, nargs=6, default=None, help='向きの基準にする姿勢（既定は --q）')
    p.add_argument('--dist', type=float, default=0.22, help='カメラ→根元の距離[m]')
    p.add_argument('--lying', action='store_true')
    p.add_argument('--pitch', type=float, default=None, help='カメラ光軸の補正[度]（正で上向き）')
    p.add_argument('--single', action='store_true', help='動かし方を必ず1関節ずつにする')
    a = p.parse_args()
    r = plan_correction(a.prefix, a.q, a.dist, a.ref or a.q, a.lying, a.pitch, a.single)
    if not r['found']:
        print('根元が見つからない:', r['reason'])
        return 1
    det = r['det']
    print(f'重ねた画像: {a.prefix}_root.png')
    print(f'根元: 面の点 ベース板{det["n_base"]} 縦板{det["n_wall"]}（2面のなす角 {det["angle_between"]:.1f}度）')
    print(f'  線の両端: {"画面の縁" if det["a_border"] else "部材の端"} / {"画面の縁" if det["b_border"] else "部材の端"}')
    print(f'  狙う点: カメラから {r["range"]:.3f}m、光軸から {r["off_deg"]:.1f}度')
    print(f'  機体座標 ({r["target_trunk"][0]:+.3f}, {r["target_trunk"][1]:+.3f}, {r["target_trunk"][2]:+.3f})')
    print(f'解き直した姿勢: {" ".join(f"{v:g}" for v in r["new_q"])}'
          f'（モデル上 光軸のずれ {r["model_aim_deg"]:.1f}度・距離 {r["model_dist"]:.3f}m）')
    if r.get('moves') is None:
        print('  動かさない:', r['reason'])
        return 1
    for m in r['moves']:
        print('  set_joints ' + ' '.join(f'{v:g}' for v in m))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
