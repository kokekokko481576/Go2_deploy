#!/usr/bin/env python3
"""明日（2026-09-29）の配置でマーカー接近を多数回まわし、止まる位置のばらつきと部材との干渉を見る。

**ROSも実機も要らない。** 制御則は実機と同じ `marker_approach.turn_drive_turn`、
機体の動きは sim_approach.py の実測モデル（指令遅れ・下限速度・直進のずれ・停止後の揺れ）。

配置（inspect_run.py の撮影位置に合わせる）:
  - 撮影位置: 部材の手前の長辺に右足を沿わせ、すき間20cm。右前足の軸（前の股関節 x=+0.193）を
    部材の左前の角に合わせる。機体の中心線は部材の縁から 0.142+0.20=0.342m 外側
  - マーカー: 撮影位置の機体の正面 standoff(0.65m)、面は機体へ向ける
  - 出発: マーカーから約1.8m手前の同じ線上、マーカーを向いて。置き方のずれを振る

座標（このスクリプト内）: マーカーが原点、面の法線が +x（機体はマーカーを -x 向きに見る）。
撮影位置は (0.65, 0)・向き -x。機体の右は +y なので、部材は y = +0.342〜+0.642、
x = 0.65-0.193 = 0.457（左端）〜 1.157。

合否（撮影位置からのずれ。写真から決めた目安）:
  - 横（部材に近づく/離れる向き）±5cm: 5cm遠い止まり方でもビードは画面内だった（2026-09-28 17:02）
  - 前後 ±15cm: 撮る場所がビードに沿ってずれるだけ（どこを撮ってもよい）。340mm区間の中なら可
  - 向き ±8度
  - 部材との最小すき間 > 0（胴体と脚の外形を長方形で近似）
"""
import argparse
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_approach as S  # noqa: E402
from sim_approach import Go2Sim, observe, wrap  # noqa: E402
from marker_approach.turn_drive_turn import Params, TurnDriveTurn  # noqa: E402

GAP = 0.20
LEG_Y = 0.142
FRONT_HIP = 0.193
MEMBER_LEN, MEMBER_DEPTH = 0.70, 0.30
STANDOFF = 0.65
# 機体の外形（trunk原点から）。前は頭まで、横は足先まで
BODY_X = (-0.33, 0.38)
BODY_Y = 0.17
TOL_LAT, TOL_ALONG, TOL_YAW = 0.05, 0.15, math.radians(8.0)


def member_rect():
    y0 = LEG_Y + GAP
    x0 = STANDOFF - FRONT_HIP
    return (x0, x0 + MEMBER_LEN, y0, y0 + MEMBER_DEPTH)


def clearance(x, y, yaw):
    """機体の外形と部材の最小距離[m]。負なら重なっている。"""
    mx0, mx1, my0, my1 = member_rect()
    c, s = math.cos(yaw), math.sin(yaw)
    best = 9.0
    for u in (BODY_X[0], (BODY_X[0] + BODY_X[1]) / 2, BODY_X[1]):
        for v in (-BODY_Y, 0.0, BODY_Y):
            if v == 0.0 and u not in BODY_X:
                continue
            px, py = x + c * u - s * v, y + s * u + c * v
            dx = max(mx0 - px, 0.0, px - mx1)
            dy = max(my0 - py, 0.0, py - my1)
            inside = mx0 <= px <= mx1 and my0 <= py <= my1
            d = -min(px - mx0, mx1 - px, py - my0, my1 - py) if inside else math.hypot(dx, dy)
            best = min(best, d)
    # 長辺の途中も見る（角だけだと部材の角が胴体の横腹に刺さるのを見逃す）
    for k in range(1, 8):
        u = BODY_X[0] + (BODY_X[1] - BODY_X[0]) * k / 8
        for v in (-BODY_Y, BODY_Y):
            px, py = x + c * u - s * v, y + s * u + c * v
            inside = mx0 <= px <= mx1 and my0 <= py <= my1
            dx = max(mx0 - px, 0.0, px - mx1)
            dy = max(my0 - py, 0.0, py - my1)
            d = -min(px - mx0, mx1 - px, py - my0, my1 - py) if inside else math.hypot(dx, dy)
            best = min(best, d)
    return best


def run_one(dist, lat, yaw_off_deg, params, seed):
    """マーカーから dist 手前・横に lat ずれた位置から、yaw_off だけ向きをずらして出発する。"""
    rng = random.Random(seed + 1)
    marker, normal_dir = (0.0, 0.0), 0.0
    px, py = dist, lat
    yaw = wrap(math.atan2(-py, -px) + math.radians(yaw_off_deg))
    sim = Go2Sim(px, py, yaw, seed=seed)
    ctl = TurnDriveTurn(params)
    t = 0.0
    ctl.start(t)
    dt, control_dt, cam_dt = 0.01, 0.05, 1.0 / S.CAM_FPS
    next_control = next_frame = 0.0
    obs, last_obs_t = None, None
    min_clear = clearance(sim.x, sim.y, sim.yaw)
    outcome, detail = None, ''
    while t < S.MAX_RUNTIME:
        if t >= next_frame:
            next_frame += cam_dt
            moving = abs(sim.vx) > 0.02 or abs(sim.wz) > 0.05
            o = observe(sim, marker, normal_dir, rng, moving, t)
            if o is not None:
                obs, last_obs_t = o, t
        if t >= next_control:
            next_control += control_dt
            stale = last_obs_t is None or t - last_obs_t > S.LOST_TIMEOUT
            if stale and (last_obs_t is None or not ctl.marker_loss_tolerable()
                          or t - last_obs_t > params.final_lost_timeout):
                outcome, detail = '失敗', f'マーカーを見失った（t={t:.1f}s、{ctl.state}）'
                break
            cmd = ctl.step(t, obs[0], obs[1], None if stale else obs[2], obs[3], None,
                           obs_time=last_obs_t)
            if stale and not cmd.done and not ctl.marker_loss_tolerable():
                outcome, detail = '失敗', f'見失ったまま {cmd.state} に移ろうとした'
                break
            if cmd.done:
                outcome = '到達' if cmd.success else '打ち切り'
                detail = cmd.reason
                break
            sim.command(t, cmd.vx, cmd.wz)
        sim.advance(t, dt)
        min_clear = min(min_clear, clearance(sim.x, sim.y, sim.yaw))
        t += dt
    else:
        outcome, detail = '失敗', '時間切れ'
    # 惰性が収まるまで流す（停止指令のあとも実機は動く）
    for _ in range(150):
        sim.advance(t, dt)
        min_clear = min(min_clear, clearance(sim.x, sim.y, sim.yaw))
        t += dt
    # 撮影位置 (0.65, 0)・向き -x（=π）からのずれ。機体の向きで分解する
    ex, ey = sim.x - STANDOFF, sim.y
    along = -ex            # 機体の前方(-x)が正
    lateral = ey           # 機体の右(+y)が正＝部材へ近づく向き
    dyaw = wrap(sim.yaw - math.pi)
    ok = (outcome == '到達' and abs(lateral) <= TOL_LAT and abs(along) <= TOL_ALONG
          and abs(dyaw) <= TOL_YAW and min_clear > 0.0)
    return dict(outcome=outcome, detail=detail, along=along, lateral=lateral, dyaw=dyaw,
                min_clear=min_clear, t=t, ok=ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', type=int, default=200)
    ap.add_argument('--lat', type=float, default=0.05, help='出発位置の横ずれの幅[m]（±）')
    ap.add_argument('--yaw', type=float, default=10.0, help='出発時の向きのずれの幅[度]（±）')
    ap.add_argument('--dist', type=float, nargs=2, default=(1.6, 2.0))
    ap.add_argument('--no-sway', action='store_true')
    ap.add_argument('--turn-tol', type=float, default=None, help='turn_tolerance[度]（既定は制御則の8度）')
    ap.add_argument('--drift', type=float, default=None,
                    help='直進1mあたりの向きの流れ[rad/m]（左が正）。既定は sim_approach の 0.025。'
                         '9/29 実機1回目は約1mで約16度＝約0.28')
    ap.add_argument('--redirect', type=float, default=None, help='redirect_tolerance[度]（既定は約15度）')
    ap.add_argument('--los', action='store_true',
                    help='視線接近（use_normal:=false）。法線を使わずマーカーの正面 standoff を狙う')
    a = ap.parse_args()
    if a.no_sway:
        S.SWAY_POS = 0.0
    # 実機の approach_real.launch.py と同じ上書き（turn_lead_deg=6.9）
    params = Params(standoff=STANDOFF, camera_x=S.CAM_X, camera_y=0.0,
                    turn_lead_angle=math.radians(6.9), use_normal=not a.los)
    if a.drift is not None:
        S.HEADING_DRIFT = a.drift
    if a.redirect is not None:
        params.redirect_tolerance = math.radians(a.redirect)
    if a.turn_tol is not None:
        params.turn_tolerance = math.radians(a.turn_tol)
        print(f'turn_tolerance を {a.turn_tol}度 に変更')
    rng = random.Random(7)
    res = []
    for k in range(a.n):
        d = rng.uniform(*a.dist)
        lat = rng.uniform(-a.lat, a.lat)
        yo = rng.uniform(-a.yaw, a.yaw)
        r = run_one(d, lat, yo, params, seed=k)
        r.update(dist=d, lat0=lat, yaw0=yo)
        res.append(r)
    n = len(res)
    ok = sum(r['ok'] for r in res)
    arrived = [r for r in res if r['outcome'] == '到達']
    print(f'接近: {"視線接近（use_normal:=false）" if a.los else "法線接近（既定）"}')
    print(f'配置: 部材とのすき間 {GAP * 100:.0f}cm / マーカーは撮影位置の正面 {STANDOFF}m')
    print(f'出発: マーカーから {a.dist[0]}〜{a.dist[1]}m、横 ±{a.lat * 100:.0f}cm、向き ±{a.yaw:.0f}度'
          f'（{n}本、停止後の揺れ{"なし" if a.no_sway else "あり"}）\n')
    print(f'撮影位置に収まった: {ok}/{n}（横±{TOL_LAT * 100:.0f}cm・前後±{TOL_ALONG * 100:.0f}cm・'
          f'向き±{math.degrees(TOL_YAW):.0f}度・部材に当たらない）')
    from collections import Counter
    print('終わり方:', dict(Counter(r['outcome'] for r in res)))

    def pct(v, q):
        v = sorted(v)
        return v[min(len(v) - 1, int(q * len(v)))]
    if arrived:
        for key, name, unit, f in (('lateral', '横（+で部材へ近い）', 'cm', 100),
                                   ('along', '前後（+で前へ行き過ぎ）', 'cm', 100),
                                   ('dyaw', '向き', '度', 180 / math.pi)):
            v = [r[key] * f for r in arrived]
            print(f'  {name:<16} 中央 {pct(v, .5):+6.1f}{unit}  5〜95% {pct(v, .05):+6.1f}〜{pct(v, .95):+6.1f}{unit}'
                  f'  最大|{max(abs(x) for x in v):.1f}|{unit}')
    mc = [r['min_clear'] * 100 for r in res]
    print(f'  部材との最小すき間  最小 {min(mc):.1f}cm  5% {pct(mc, .05):.1f}cm  中央 {pct(mc, .5):.1f}cm'
          f'  （重なった本: {sum(1 for x in mc if x <= 0)}）')
    bad = [r for r in res if not r['ok']]
    if bad:
        print('\n外れた例（最大5本）:')
        for r in bad[:5]:
            print(f"  出発 {r['dist']:.2f}m 横{r['lat0'] * 100:+.0f}cm 向き{r['yaw0']:+.0f}度 → {r['outcome']} "
                  f"横{r['lateral'] * 100:+.1f}cm 前後{r['along'] * 100:+.1f}cm 向き{math.degrees(r['dyaw']):+.1f}度 "
                  f"すき間{r['min_clear'] * 100:.1f}cm  {r['detail'][:60]}")


if __name__ == '__main__':
    main()
