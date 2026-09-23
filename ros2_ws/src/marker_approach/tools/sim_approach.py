#!/usr/bin/env python3
"""turn-drive-turn 接近制御の**実機なし検証**（tools/sim_approach.py）。

`marker_approach.turn_drive_turn` の制御則だけを取り出し、Go2の実測特性を入れた
運動モデルで回す。ROSもGo2も要らないので、機体が無い日に制御則を詰められる。

模擬している実測値（すべて2026-09-02の実機計測。出典は README と memory）:

  - **歩容の下限速度**: 0.20m/s 未満の指令では脚が踏み出さない（2026-09-23に更新）
  - **前進/旋回の応答は一次式**: v = 1.0824*cmd - 0.1517 / w = 0.5699*cmd - 0.1572
  - **指令の遅れは前進0.42s・旋回0.77s**: 行き過ぎ 0.120m と 18.3度 を生む
  - **視野 水平±46度**、150mmタグの実用距離 3m
  - **姿勢の曖昧性**: 視線と法線のなす角 0度→1.8 / 7度→1.1 / 31度→44。
    曖昧性が低いと法線の方位が±7度暴れる
  - カメラ位置 base_link 前方327.15mm

**このシミュレータで分からないこと**（実機で確かめる必要がある）:
  旋回の行き過ぎ量（`turn_lead_angle` は未実測で0のまま）、
  歩容の横滑り、床の摩擦差、検出の脱落、機体の揺れによる観測ノイズの実分布。

使い方:
  python3 tools/sim_approach.py            # 既定シナリオ一式
  python3 tools/sim_approach.py --trace    # 1周期ごとの状態も出す
  python3 tools/sim_approach.py --line-of-sight   # 「マーカーの手前まで行く」モード
"""
import argparse
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from marker_approach.turn_drive_turn import Params, TurnDriveTurn, wrap  # noqa: E402

# --- 実機の特性（実測） ---
CAM_X = 0.333            # base_link -> カメラ 前方[m]（2026-09-23 実機実測）
FOV_HALF = math.radians(46.0)
MAX_RANGE = 3.0          # 150mmタグの実用距離[m]
MAX_INCIDENCE = math.radians(70.0)   # これ以上斜めだと四角形が潰れて検出が落ちる
# --- 指令→実速度（2026-09-23 実機実測で全面改訂）---
# **効率を掛ける形では合わない。** 実測は一次式に乗る（残差: 前進12mm/s・旋回0.02rad/s以下）。
#   前進 指令:実速度 = 0.25:0.107 / 0.30:0.181 / 0.35:0.234 / 0.40:0.285 / 0.50:0.383 [m/s]
#   旋回 指令:実角速度 = 0.40:0.060 / 0.60:0.205 / 0.80:0.290 / 1.00:0.412 [rad/s]
#     （旋回は「総方位変化 - 行き過ぎ」を指令2秒で割った値）
# 改訂前は 前進下限0.10・効率0.83、旋回下限0.20・効率0.80 だったが、
# **どちらも実機より甘い**。0.20m/s の前進指令は実際には踏み出さない（#74 の正体）。
WALK_DEADBAND = 0.20     # これ未満の vx 指令では進まない[m/s]（実測: 0.20で実動率34%）
WALK_GAIN, WALK_OFFSET = 1.0824, 0.1517      # v = GAIN*cmd - OFFSET（ゼロ交差 0.140）
TURN_DEADBAND = 0.35     # これ未満の wz 指令は実用にならない[rad/s]（ゼロ交差 0.276）
TURN_GAIN, TURN_OFFSET = 0.5699, 0.1572      # w = GAIN*cmd - OFFSET
# **遅れは前進と旋回で倍違う。** 停止の行き過ぎ(実測0.120m @0.285m/s)からは0.42秒、
# 旋回の行き過ぎ(実測18.3度 @0.412rad/s)からは0.77秒。1つの値では両方を再現できない。
COMMAND_LATENCY = 0.42       # 前進[s]
TURN_LATENCY = 0.77          # 旋回[s]
HEADING_DRIFT = 0.025    # 直進1mあたりの向きのずれ[rad]（実測: 3.5mで横0.15m）
CAM_FPS = 14.4
LOST_TIMEOUT = 0.5
MAX_RUNTIME = 90.0


def ambiguity_of(tilt):
    """視線とマーカー法線のなす角[rad]から曖昧性を返す（実測3点に合わせた当てはめ）。"""
    t = math.degrees(abs(tilt))
    return 1.1 + 42.9 * (max(0.0, t - 7.0) / 24.0) ** 2


class Go2Sim:
    """前進と旋回だけの2次元モデル。指令の遅れと下限速度・効率を入れる。"""

    def __init__(self, x, y, yaw, seed=0):
        self.x, self.y, self.yaw = x, y, yaw
        self.queue = []          # (適用時刻, vx)   前進
        self.wqueue = []         # (適用時刻, wz)   旋回。遅れが前進と違うので別に持つ
        self.vx = self.wz = 0.0
        self.rng = random.Random(seed)
        self.path = 0.0

    def command(self, now, vx, wz):
        self.queue.append((now + COMMAND_LATENCY, vx))
        self.wqueue.append((now + TURN_LATENCY, wz))

    @staticmethod
    def _respond(cmd, deadband, gain, offset):
        if abs(cmd) < deadband:
            return 0.0
        return math.copysign(max(0.0, gain * abs(cmd) - offset), cmd)

    def advance(self, now, dt):
        while self.queue and self.queue[0][0] <= now:
            _, vx = self.queue.pop(0)
            self.vx = self._respond(vx, WALK_DEADBAND, WALK_GAIN, WALK_OFFSET)
        while self.wqueue and self.wqueue[0][0] <= now:
            _, wz = self.wqueue.pop(0)
            self.wz = self._respond(wz, TURN_DEADBAND, TURN_GAIN, TURN_OFFSET)
        ds = self.vx * dt
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)
        self.yaw = wrap(self.yaw + self.wz * dt + HEADING_DRIFT * ds)
        self.path += abs(ds)


def observe(sim, marker, normal_dir, rng):
    """カメラから見えるか判定し、見えていれば base_link 座標系の観測を返す。"""
    mx_w, my_w = marker
    cam = (sim.x + CAM_X * math.cos(sim.yaw), sim.y + CAM_X * math.sin(sim.yaw))
    dx, dy = mx_w - cam[0], my_w - cam[1]
    rng_cam = math.hypot(dx, dy)
    bearing_cam = wrap(math.atan2(dy, dx) - sim.yaw)
    # マーカーから見たカメラの方向と法線のなす角（斜めすぎると検出できない）
    tilt = abs(wrap(math.atan2(-dy, -dx) - normal_dir))
    if abs(bearing_cam) > FOV_HALF or rng_cam > MAX_RANGE or tilt > MAX_INCIDENCE:
        return None
    amb = ambiguity_of(tilt)
    # base_link 座標系でのマーカー位置（ノードが算出するのと同じ量）
    bx, by = mx_w - sim.x, my_w - sim.y
    c, s = math.cos(-sim.yaw), math.sin(-sim.yaw)
    mx = c * bx - s * by + rng.gauss(0.0, 0.002)
    my = s * bx + c * by + rng.gauss(0.0, 0.002)
    # 法線の方位（ロボット側を向く向き）。曖昧性が低いと大きく暴れる
    sigma = math.radians(8.0 / amb)
    gamma = wrap(normal_dir - sim.yaw + rng.gauss(0.0, sigma))
    return mx, my, gamma, amb


def run(dist, alpha_deg, yaw_off_deg, params, seed=0, trace=False):
    """マーカーから距離 dist、法線から alpha だけずれた位置に置いて回す。"""
    rng = random.Random(seed + 1)
    marker, normal_dir = (0.0, 0.0), 0.0     # 法線は世界座標の +x 向き
    a = math.radians(alpha_deg)
    # ロボットは法線から alpha だけずれた方向、距離 dist の位置。初期はマーカーを向く
    px, py = dist * math.cos(a), dist * math.sin(a)
    yaw = wrap(math.atan2(-py, -px) + math.radians(yaw_off_deg))
    sim = Go2Sim(px, py, yaw, seed=seed)

    ctl = TurnDriveTurn(params)
    t = 0.0
    ctl.start(t)
    dt, control_dt, cam_dt = 0.01, 0.05, 1.0 / CAM_FPS
    next_control = next_frame = 0.0
    obs = None
    last_obs_t = None
    last_self_err = float('nan')
    vx = wz = 0.0
    outcome, detail = None, ''

    while t < MAX_RUNTIME:
        if t >= next_frame:
            next_frame += cam_dt
            o = observe(sim, marker, normal_dir, rng)
            if o is not None:
                obs, last_obs_t = o, t

        if t >= next_control:
            next_control += control_dt
            # **真横へ旋回する区間だけは見失いで落とさない。**
            # 90度回せばマーカーは必ず視野(±46度)の外に出る。この区間はヨー角で
            # 開ループに回しているので、見えなくても進行できる（実機ノード側も同じ扱いが要る）。
            if not ctl.marker_optional() and (last_obs_t is None or t - last_obs_t > LOST_TIMEOUT):
                outcome, detail = '失敗', f'マーカーを{LOST_TIMEOUT}s見失った（t={t:.1f}s）'
                break
            # ヨー角の観測。実機ならオドメトリ/IMU。開始からの差分しか使わないので
            # 絶対の基準はどうでもよいが、ノイズは載せておく
            # 推測航法の観測 (x, y, yaw)。**位置も要る**（その場旋回でも機体は動く）
            odom_obs = (sim.x + rng.gauss(0.0, 0.003),
                        sim.y + rng.gauss(0.0, 0.003),
                        sim.yaw + rng.gauss(0.0, math.radians(0.5)))
            cmd = ctl.step(t, obs[0], obs[1], obs[2], obs[3], odom_obs)
            vx, wz = cmd.vx, cmd.wz
            last_self_err = cmd.pos_err
            if trace:
                print(f'  t={t:5.2f} {cmd.state:<8} vx={vx:+.3f} wz={wz:+.3f} '
                      f'誤差{cmd.pos_err * 1000:5.0f}mm 方位{math.degrees(cmd.bearing):+6.1f}度 '
                      f'カメラ方位{math.degrees(cmd.bearing_cam):+6.1f}度 '
                      f'ゴール方位{math.degrees(cmd.goal_bearing):+6.1f}度 '
                      f'alpha={math.degrees(cmd.alpha):+5.1f}'
                      f'{"" if cmd.alpha_trusted else "?"} 曖昧性{obs[3]:.1f}')
            if cmd.done:
                outcome = '成功' if cmd.success else '打ち切り'
                detail = cmd.reason
                break
            sim.command(t, vx, wz)

        sim.advance(t, dt)
        t += dt
    else:
        outcome, detail = '失敗', f'{MAX_RUNTIME}s で収束しなかった'

    # 真値での評価。ゴールは法線上 standoff の点。正対＝機体の向きがマーカーを指すこと
    gx_true = marker[0] + math.cos(normal_dir) * params.standoff
    gy_true = marker[1] + math.sin(normal_dir) * params.standoff
    pos_err = math.hypot(sim.x - gx_true, sim.y - gy_true)
    # 最終姿勢の誤差。**final_heading によって「正しい向き」が変わる**。
    # marker=マーカーを正面(0度)、right=マーカーが右真横(-90度)、left=左真横(+90度)。
    #
    # ただし**真横へ回るのは到達に成功したときだけ**である（打ち切ったときは
    # マーカーを正面に残す。見失った状態で終わらせないための設計）。
    # したがって採点も「その回が実際に狙った姿勢」に対して行う。狙っていない姿勢からの
    # 90度ずれを誤差として数えると、打ち切りが全部不合格に見えてしまい判断を誤る。
    side_done = params.final_heading != 'marker' and ctl.arrival is not None and ctl.success
    desired = 0.0
    if side_done:
        desired = (-params.side_turn_angle if params.final_heading == 'right'
                   else +params.side_turn_angle)
    bearing_true = wrap(math.atan2(marker[1] - sim.y, marker[0] - sim.x) - sim.yaw)
    head_err = wrap(bearing_true - desired)
    # 法線からの残ずれ（真値）。「正対」の精度はこれで見る。alpha が観測できない配置では
    # 制御則は自分の誤差をゼロと信じたまま、これが残る
    alpha_final = wrap(math.atan2(sim.y - marker[1], sim.x - marker[0]) - normal_dir)
    segments = sum(1 for e in ctl.events if e[1] in (ctl.TURN_TO_GOAL, ctl.DRIVE, ctl.FINAL_TURN))
    return dict(outcome=outcome, detail=detail, t=t, pos_err=pos_err, head_err=head_err,
                side_done=side_done,
                alpha_final=alpha_final, self_err=last_self_err,
                dist_to_marker=math.hypot(sim.x, sim.y), segments=segments,
                path=sim.path, cycles=ctl.cycles, events=ctl.events)


# 任務としての合否。制御則の自己申告ではなく**真値**で見る。
#
# 位置の合格線は「法線が観測できない床」で決まる。150mmタグの姿勢は2解あり、
# 視線と法線のなす角が12度を下回ると2解が拮抗して法線の向きが分からなくなる
# （曖昧性 < 2.0）。つまり**法線から12度以内に入った時点でそれ以上寄れない**。
# 残る横ずれは standoff x sin(12度)。ここに区間制御の分解能60mmを足した値を線にする。
AMBIGUITY_FLOOR_DEG = 12.0
PASS_HEAD = math.radians(6.0)   # 真の方位誤差[rad]。旋回の分解能(約4度)より少し緩く

# **任務基準の合否（2026-09-23追加）。** 上の PASS_HEAD=6度 は旋回分解能が4度だった頃の値で、
# 実機の旋回の行き過ぎ（min_wz で7度）に対して厳しすぎる。
# 本当に効くのは「**ビードがアームの届く範囲に入るか**」なので、そちらで判定する。
#
#   ビード位置の誤差 ≒ 機体の位置誤差 + BEAD_LEVER × 方位誤差  ≤ BEAD_BUDGET
#
# 機体が方位を誤ると、ビードは機体から見て BEAD_LEVER の腕で振れる。
BEAD_LEVER = 0.375    # 撮影位置でのビードまでの横距離[m]（アームの最良点）
BEAD_BUDGET = 0.175   # そこでの余裕[m]（3軸・床+0.08m・胴体の占有を除外した値）

SCENARIOS = [
    # (マーカーからの距離[m], 法線からのずれalpha[度], 初期の向きのずれ[度], 説明)
    (1.85, 0, 0, '正対から1.85m（2026-09-02の実機成功例と同条件）'),
    (1.85, 20, 0, '斜め20度から'),
    (1.85, -30, 0, '斜め-30度から'),
    (1.85, 30, 0, '斜め+30度から（視野の限界付近）'),
    (2.50, 25, 0, '遠め2.5m・斜め25度'),
    (1.00, 15, 0, '近め1.0m・斜め15度'),
    (1.85, 20, -25, '斜め20度、初期の向きも25度ずれ'),
    (1.20, -35, 15, '斜め-35度（法線がよく見える）'),
    (0.80, 10, 0, 'standoff(0.65m)のすぐ外0.80m'),
    (0.55, 0, 0, 'standoffより内側0.55m（後退が要る）'),
]


# 視線接近の合格線。マーカーからの距離の許容[m]（区間制御の分解能60mm + 停止の行き過ぎ）
LOS_PASS_RANGE = 0.09


def bead_error(r):
    """機体の誤差を、ビード位置の誤差[m]に換算する。合否はこれで見る。"""
    return r['pos_err'] + BEAD_LEVER * abs(r['head_err'])


def passed(r, params, pass_pos):
    """任務としての合否。**モードで採点対象が変わる**（狙っていない量で落とさない）。"""
    if r['outcome'] == '失敗' or bead_error(r) > BEAD_BUDGET:
        return False
    if params.use_normal:
        return r['pos_err'] <= pass_pos
    return abs(r['dist_to_marker'] - params.standoff) <= pass_pos


def sweep(params, seeds, pass_pos):
    """距離と法線からのずれ(alpha)の格子で回し、届く範囲を出す。

    「法線から何度までなら寄れるか」は視野(±46度)で決まる幾何の話で、
    ゲイン調整では動かない。数字で押さえておくと運用の前提を決められる。
    """
    dists = [1.0, 1.5, 2.0, 2.5]
    alphas = [-40, -30, -20, -10, 0, 10, 20, 30, 40]
    if params.use_normal:
        print('真の位置誤差[mm]（括弧は残alpha[度]。* は不合格）')
    else:
        # 視線接近では法線上の点を狙っていない。合否はマーカーからの距離で見た上で、
        # **表には「法線上の点からどれだけ離れて止まったか」＝このモードの代償**を出す
        print('［視線接近］法線上の点からの距離[mm]（括弧は残alpha[度]。* は不合格）')
        print('  ＝このモードで捨てている量。合否はマーカーからの距離と方位で判定している')
    print('       ' + ''.join(f'{a:>+13d}度' for a in alphas))
    for d in dists:
        row = [f'{d:.1f}m  ']
        for al in alphas:
            errs, res, ok_all = [], [], True
            for s in range(seeds):
                r = run(d, al, 0, params, seed=s * 17)
                errs.append(r['pos_err'] * 1000)
                res.append(math.degrees(r['alpha_final']))
                ok_all &= passed(r, params, pass_pos)
            m = sum(errs) / len(errs)
            a_m = sum(res) / len(res)
            row.append(f'{m:>7.0f}({a_m:+4.0f}){"*" if not ok_all else " "}')
        print(''.join(row))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trace', action='store_true')
    ap.add_argument('--seeds', type=int, default=3, help='シナリオごとの試行数')
    ap.add_argument('--standoff', type=float, default=0.65)
    ap.add_argument('--turn-lead-deg', type=float, default=None,
                    help='旋回の行き過ぎ補償[度]。既定は制御則側の値を使う')
    ap.add_argument('--only', type=int, default=None, help='シナリオ番号だけ実行')
    ap.add_argument('--sweep', action='store_true', help='距離とalphaの格子で届く範囲を出す')
    ap.add_argument('--final-heading', choices=['marker', 'right', 'left'], default='marker',
                    help='到達後の向き。right=マーカーを右真横に見る姿勢まで旋回する')
    ap.add_argument('--line-of-sight', action='store_true',
                    help='ゴールを法線上ではなく**視線上**に置く（＝マーカーの手前まで行く）。'
                         '合否も「マーカーからの距離が standoff か」で判定する')
    a = ap.parse_args()

    params = Params(standoff=a.standoff, camera_x=CAM_X, camera_y=0.0,
                    use_normal=not a.line_of_sight, final_heading=a.final_heading)
    if a.turn_lead_deg is not None:
        params.turn_lead_angle = math.radians(a.turn_lead_deg)
    bad = params.validate()
    if bad:
        print('パラメータが不整合です:')
        for b in bad:
            print(' -', b)
        return 1

    if params.final_heading != 'marker':
        print(f'[最終姿勢] マーカーが{"右" if params.final_heading == "right" else "左"}真横'
              f'（{math.degrees(params.side_turn_angle):.0f}度）に来るまで、到達後に'
              'その場旋回する。**この区間はマーカーが視野外なのでヨー角で開ループ**')
    print(f'standoff={params.standoff}m  到達判定={params.pos_tolerance * 1000:.0f}mm / '
          f'{math.degrees(params.ang_tolerance):.1f}度  '
          f'turn_lead={math.degrees(params.turn_lead_angle):.1f}度  '
          f'試行={a.seeds}回/シナリオ')
    if params.use_normal:
        pass_pos = params.standoff * math.sin(math.radians(AMBIGUITY_FLOOR_DEG)) + 0.06
        print(f'合否は真値で判定: **ビード位置の誤差** '
              f'（機体の位置誤差 + {BEAD_LEVER:.3f}m × 方位誤差）が '
              f'{BEAD_BUDGET * 1000:.0f}mm 以内。'
              f'アームが吸収できる量で決めている。参考: 位置だけなら {pass_pos * 1000:.0f}mm'
              f'（法線が観測できない床 standoff x sin{AMBIGUITY_FLOOR_DEG:.0f}度'
              f'={params.standoff * math.sin(math.radians(AMBIGUITY_FLOOR_DEG)) * 1000:.0f}mm'
              f' + 区間制御の分解能60mm）。'
              f'見失い・時間切れは無条件で否\n')
    else:
        # 視線接近では**法線上の点を狙っていない**ので、そこからの距離で採点しない。
        # 任務は「マーカーの正面 standoff まで行って向く」なので、その2つで見る。
        pass_pos = LOS_PASS_RANGE
        print(f'[視線接近] ゴールはマーカーの手前（視線上 {params.standoff}m）。'
              f'合否は真値で判定: マーカーからの距離が {params.standoff * 1000:.0f}'
              f'±{pass_pos * 1000:.0f}mm かつ '
              f'方位 {math.degrees(PASS_HEAD):.0f}度 以内。'
              f'**法線ずれ(残alpha)は評価しない**（このモードでは残るのが仕様）\n')
    if a.sweep:
        sweep(params, a.seeds, pass_pos)
        return 0
    header = (f'{"#":>2} {"シナリオ":<30} {"判定":<4} {"結果":<6} {"時間":>6} '
              f'{"真の誤差" if params.use_normal else "距離のずれ":>9} '
              f'{"自己申告":>9} {"残alpha":>8} {"方位":>7} {"姿"}{"区間":>4} {"周":>3}')
    print(header)
    print('-' * len(header))
    all_ok = True
    results = []
    for i, (d, alpha, yaw_off, note) in enumerate(SCENARIOS):
        if a.only is not None and a.only != i:
            continue
        for s in range(a.seeds):
            if a.trace:
                print(f'\n[{i}] {note} seed={s}')
            r = run(d, alpha, yaw_off, params, seed=s * 17, trace=a.trace)
            ok = passed(r, params, pass_pos)
            all_ok &= ok
            shown = (r['pos_err'] if params.use_normal
                     else abs(r['dist_to_marker'] - params.standoff))
            print(f'{i:>2} {note[:30]:<30} {"合" if ok else "否":<4} {r["outcome"]:<6} {r["t"]:5.1f}s '
                  f'{shown * 1000:7.0f}mm {r["self_err"] * 1000:7.0f}mm '
                  f'{math.degrees(r["alpha_final"]):+6.1f}度 '
                  f'{math.degrees(r["head_err"]):+5.1f}度 '
                  f'{"横" if r["side_done"] else "正"}'
                  f'{r["segments"]:>4} {r["cycles"] + 1:>3}'
                  + ('' if ok else f'\n     -> {r["detail"]}'))
            results.append((i, note, ok, r))
    hard = [r for _, _, _, r in results if r['outcome'] == '失敗']
    ng = [(i, n, r) for i, n, ok, r in results if not ok]
    print(f'\n{len(results) - len(ng)}/{len(results)} 合格'
          f'（見失い・時間切れ {len(hard)} 件）')
    if ng:
        print('不合格:')
        for i, n, r in ng:
            print(f'  [{i}] {n} 位置{r["pos_err"] * 1000:.0f}mm '
                  f'マーカーから{r["dist_to_marker"] * 1000:.0f}mm '
                  f'方位{math.degrees(r["head_err"]):+.1f}度 残alpha{math.degrees(r["alpha_final"]):+.1f}度')
    return 0 if all_ok else 2


if __name__ == '__main__':
    sys.exit(main())
