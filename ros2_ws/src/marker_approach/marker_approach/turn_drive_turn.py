"""turn-drive-turn 方式の接近制御則（**ROS非依存**。単体でシミュレーション検証できる）。

前身は vx/vy/wz を同時に出す全方向制御だったが、**Go2の横移動(vy)は使えない**ため
作り直した。実測（2026-09-02、4秒指令の実効率）:

    vx=0.15 → 83% / vy=0.15 → 1.3% / vy=0.20 → 7.7%

しかも `vy=-0.056` 程度が出ているだけで前進そのものが止まる（41秒で127mm）。
同じ条件で `max_vy:=0.0` にすると1652mmを11秒で走り切った。
つまり **横移動は「効かない」ではなく「前進を殺す」**。混ぜてはいけない。

## 構成

前進(vx)と旋回(wz)を**同時に出さず、区間に分ける**。

    TURN_TO_GOAL  その場旋回。ゴール（マーカー法線上のstandoff点）を向く
    DRIVE         直進のみ（wz=0）。ゴールの手前 stop_lead_distance で止める
    FINAL_TURN    その場旋回。マーカーを正面に入れる（＝法線上にいるので正対になる）
    CHECK         測り直して、残差が許容内なら完了。駄目なら TURN_TO_GOAL から繰り返す

各区間の切り替わりでは必ず **SETTLE**（ゼロ指令で settle_time 秒静止）を挟む。
四足は停止指令後も踏み出し中の歩を完了させ（実測の行き過ぎ約47mm）、
静止指令でも揺れる（距離測定の標準偏差が0.2mm→15mmに跳ねる）。
**動揺中の観測で次の区間を決めてはいけない。**

旋回はオドメトリを使わず、毎周期その時点の画像から見えるゴール方位をゼロに近づける
**視覚のクローズドループ**である。角度を積分しないので機体の旋回効率のばらつきに強い。

## マーカーを視野から外さない（**この方式の一番の難所**。設計の理由を残す）

Go2前方カメラの視野は水平±46度しかない。ここを外すと制御不能になる。
turn-drive-turn では次の2つが視野を食う。

1. **ゴールを向く旋回**。マーカーの方位は旋回した分だけ反対側へ寄る
2. **直進**。ゴールはマーカーの真正面ではなく法線上の点なので、進路はマーカーの
   横を通る弦になる。向きを変えずに近づく間、マーカーの方位は**単調に増える**

2が効く量は大きい。法線から30度ずれた位置から弦を直進すると、到達時点で
マーカーの方位は**約44度**になる（シミュレーションで確認。±46度をほぼ使い切る）。
20度なら約30度、35度なら約50度で**視野から出る**。

**視野の制約は base_link ではなくカメラ位置で効く。** カメラは base_link 前方327mmに
あり、近距離では同じ機体姿勢でもカメラから見た方位のほうが大きくなる（実例: 機体から
0.9m・方位29度のマーカーは、カメラから見ると方位44度）。**判定はカメラ座標系で行う。**

対策は**弦を1本で走らず折れ線に刻むこと**。刻めるのは、直進のたびに法線からのずれ
（alpha）が減り、次に必要な旋回が小さくなるからである（30度から弦の半分を走ると22度）。

  - 旋回の目標は `[bearing_cam - fov_budget, bearing_cam + fov_budget]` に切り詰める。
    ゴール方位がこの外なら、**マーカーを画面に戻す向きに旋回する**（fov_budget まで戻す）
  - 直進中にカメラ方位が `drive_bearing_limit` を超えたら区間を切り、向き直してから続ける

`drive_bearing_limit > fov_budget` でなければ、旋回した直後に区間を切って足踏みする。

ゴールが背後（|goal_bearing| > 90度）にあるときは**旋回せず後退する**。
振り向くとマーカーを確実に見失うため。後退中は視野の予算も使わない。

## 収束しないことを検知する

折れ線で刻んでも、法線から大きくずれた至近距離からは**原理的に届かない**。
（法線まで横に325mm動く必要があるのに、その方向は視線から90度＝視野外。）
1周で `min_progress` も縮まらなければ、残差を報告して打ち切る。**黙って粘らない。**

## ゴールの置き方（`use_normal`）

既定は**マーカー法線上の standoff 点**である。マーカーの面に正対して止まるので、
アームで作業対象に向かうならこれが要る。ただし法線上へ行くには横へ寄る必要があり、
**横へ寄る方向は視線から90度＝視野の外**なので、上に書いた弦の刻みで少しずつしか
寄れない。届く範囲は 2m以上からで法線±40度、1mからは±20度しかない。

`use_normal=False` にすると、ゴールを**視線上の standoff 点**＝「マーカーの手前」に
置く。横へ寄る必要が消えるので**届かない配置がなくなり**、弦にもならないので
マーカーの方位は増えない（視野の心配が要らない）。代償は姿勢で、

    法線から alpha ずれた位置から寄ると、法線上の点から 2*standoff*sin(alpha/2) 離れ、
    マーカーの面に対して alpha だけ斜めを向いたまま止まる
    （standoff 0.65m・alpha 28.6度なら 321mm 横、面に対して28.6度斜め）

**alpha の観測と報告はどちらのモードでも行う。** 使うかどうかだけが違うので、
視線接近で止まったあとのログを見れば「法線からどれだけ斜めか」は分かる。

## 姿勢の曖昧性と alpha（前身から引き継ぎ）

平面タグの姿勢は2解あり、正対に近いほど拮抗して法線が信用できない。
`ambiguity >= min_ambiguity` のときだけ法線推定を更新し、
**ロボット座標系のベクトルではなく「視線と法線のなす角 alpha」で保持する**
（その場旋回で不変な相対量。turn-drive-turn は旋回区間が長いのでこれが効く）。
"""
import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Params:
    """制御則のパラメータ。既定値は実機実測（2026-09-02）に基づく。"""

    # 幾何
    standoff = 0.65              # マーカー法線上のゴール距離[m]
    pos_tolerance = 0.06         # 到達判定[m]。四足は一歩10〜20cm。30mmでは判定に入らない
    # 正対判定[rad]。**5度より厳しくしても入れない。** 指令が実速度に現れるまで0.3sあり、
    # 最低旋回速度 min_wz では停止指令のあとも約4度回る（turn_lead_angle）。
    # 前身の実機ログの「方位誤差 -0.3度」は**停止指令の瞬間の観測値**で、
    # 機体が落ち着いたあとの値は測っていない。こちらは静止後に測り直した値を報告する。
    ang_tolerance = 0.09
    # ゲインと速度
    k_x = 0.6
    k_yaw = 1.0
    max_vx = 0.20
    max_wz = 0.40
    min_translation_speed = 0.15  # これ未満では脚が踏み出さない（0.05m/sで不動を実測）
    min_wz = 0.30
    # 区間の切り替え
    turn_tolerance = 0.09        # ゴール方位をこの範囲に入れたら旋回終了[rad]（約5度）
    redirect_tolerance = 0.26    # 直進中にゴール方位がこれを超えたら旋回に戻る[rad]（約15度）
    stop_lead_distance = 0.05    # 停止指令後の行き過ぎ分だけ手前で止める[m]（実測47mm）
    # 旋回の行き過ぎ分[rad]。停止指令のあとに回ってしまう角度だけ手前で指令を切る。
    # 既定 0.07rad(4度) は 指令遅れ0.3s × min_wz×旋回効率 からの推定で、**実機では未実測**。
    # 実機で測る手順: その場旋回を1秒だけ指令し、指令をゼロにした時刻の方位と
    # 静止後の方位を比べる（この量はシミュレータでは検証できない）。
    # 判定は `max(許容値, この値)` で行う。**足し算にしてはいけない**（許容値が惰性と
    # 同程度のとき、早く切りすぎて逆側へ行き過ぎる。シミュレーションで確認）。
    turn_lead_angle = 0.07
    settle_time = 0.7            # 区間の間に静止する時間[s]
    max_cycles = 8               # turn-drive-turn の繰り返し上限
    min_progress = 0.03          # 1周でこれだけ縮まらなければ打ち切る[m]
    # 視野の予算。**カメラ座標系での**マーカー方位で数える（base_linkではない。後述）
    fov_budget = math.radians(25.0)        # 旋回後にマーカーをこの方位内に残す
    drive_bearing_limit = math.radians(33.0)  # 直進中にこれを超えたら区間を切って向き直す
    # base_link -> カメラ の取り付け（前方・左方[m]）。視野の判定に使う
    camera_x = 0.0
    camera_y = 0.0
    # 法線推定
    min_ambiguity = 2.0
    normal_alpha = 0.3
    # ゴールをどこに置くか。True: マーカー**法線上**の standoff 点（正対して止まる）。
    # False: **視線上**の standoff 点＝「マーカーの手前」。後者は横へ寄る必要が
    # ないので届かない配置がなくなるが、**法線からのずれ alpha はそのまま残る**
    # （alpha ずれた位置から寄ると、法線上の点からは 2*standoff*sin(alpha/2) 離れて止まる）。
    # どちらでも alpha の観測と報告は行う。使うかどうかだけが違う。
    use_normal = True
    # 到達後の最終姿勢。'marker'=マーカーを正面に見て止まる（既定・従来どおり）。
    # 'right'=マーカーが**右真横**に来るまでその場旋回する（＝左へ90度回る）。'left'は逆。
    # 用途: 対象物の横に着けてから、機体側面のアームで作業する。
    # **この旋回中はマーカーは視野から出る**（視野は実機±46度、sim±31度）ので
    # カメラでは追えない。ヨー角の観測（`step` の `yaw_obs`）を使って開ループで回す。
    final_heading = 'marker'
    # 真横へ回すときの目標角[rad]。**「90度」ではなく「引きずり補正込みの値」を入れる。**
    #
    # 四足の「その場旋回」はその場ではない。Gazeboの実測で、75度回る間に機体が
    # 116mm動いた（90度なら約14cm）。standoff 0.65m ではこれがマーカー方位の
    # 約9度に相当し、90度きっちり回すと真横から9度ずれて止まる。
    # **この量は系統的**（3回で -9.4 / -9.5 / -8.5度、ばらつき1度以内）なので、
    # 目標角から差し引けば消える。Gazeboで 81度 にしたら -0.5〜-2.9度 に収まった。
    #
    # 差し引く量は「引きずりの横成分 / standoff」で決まるので、**standoff を変えたら
    # 測り直すこと**（遠いほど角度への効きは小さい）。機体が変われば当然変わる。
    # 手順: 90度で1回走らせ、止まった姿勢の方位のずれを測り、その分を引く。
    side_turn_angle = math.pi / 2

    def __init__(self, **kw):
        for k, v in kw.items():
            if not hasattr(Params, k):
                raise KeyError(f'未知のパラメータ: {k}')
            setattr(self, k, v)

    def validate(self):
        """設定の矛盾を返す（空リストなら問題なし）。"""
        bad = []
        if self.max_vx < self.min_translation_speed:
            bad.append(f'max_vx({self.max_vx}) が min_translation_speed'
                       f'({self.min_translation_speed}) より小さい。'
                       'この設定では脚が踏み出さず、ゴール手前で詰められなくなる')
        if self.max_wz < self.min_wz:
            bad.append(f'max_wz({self.max_wz}) が min_wz({self.min_wz}) より小さい')
        if self.drive_bearing_limit <= self.fov_budget:
            bad.append(f'drive_bearing_limit({math.degrees(self.drive_bearing_limit):.0f}度) が '
                       f'fov_budget({math.degrees(self.fov_budget):.0f}度) 以下。'
                       '旋回で戻した直後に区間を切ることになり、その場で足踏みする')
        if self.ang_tolerance < self.turn_lead_angle:
            bad.append(f'ang_tolerance({math.degrees(self.ang_tolerance):.1f}度) が '
                       f'turn_lead_angle({math.degrees(self.turn_lead_angle):.1f}度) より小さい。'
                       '停止判定が惰性の推定値だけで決まり、推定が外れると判定に入らない。'
                       'ang_tolerance を turn_lead_angle 以上にすること')
        if self.final_heading not in ('marker', 'right', 'left'):
            bad.append(f"final_heading({self.final_heading!r}) が不正。"
                       "'marker' / 'right' / 'left' のいずれかにすること")
        if self.pos_tolerance <= self.stop_lead_distance:
            bad.append(f'pos_tolerance({self.pos_tolerance}) が '
                       f'stop_lead_distance({self.stop_lead_distance}) 以下。'
                       '手前で止めた時点で到達判定に入らず、旋回と直進を往復する')
        return bad


class Command:
    """1周期の出力。"""

    def __init__(self, vx, wz, state, pos_err, bearing, bearing_cam, goal_bearing, dist,
                 alpha, alpha_trusted, cycles, done=False, success=False, reason=None):
        self.vx, self.wz, self.state = vx, wz, state
        self.pos_err, self.bearing, self.goal_bearing, self.dist = pos_err, bearing, goal_bearing, dist
        self.bearing_cam = bearing_cam
        self.alpha, self.alpha_trusted, self.cycles = alpha, alpha_trusted, cycles
        self.done, self.success, self.reason = done, success, reason


class TurnDriveTurn:
    """turn-drive-turn の状態機械。入力はすべて base_link 座標系。

    使い方: `start(now)` してから毎周期 `step(now, mx, my, gamma_obs, ambiguity)`。
    `gamma_obs` はマーカー面の法線（ロボット側を向くよう符号を揃えたもの）の方位[rad]。
    観測できていなければ None。
    """

    TURN_TO_GOAL = 'ゴールへ旋回'
    DRIVE = '直進'
    FINAL_TURN = '正対へ旋回'
    CHECK = '測り直し'
    SIDE_TURN = '真横へ旋回'
    SETTLE = '静止待ち'
    DONE = '完了'

    def __init__(self, params=None):
        self.p = params or Params()
        self.events = []          # (時刻, 状態, 理由) 遷移の記録。検証とログに使う
        self._reset_state()

    def _reset_state(self):
        self.state = self.TURN_TO_GOAL
        self.next_state = None
        self.settle_until = 0.0
        self.cycles = 0
        self.drive_dir = 1
        self.cycle_start_err = None   # 周の開始時のゴール誤差。進んでいるかの判定に使う
        self.abort_reason = None      # 打ち切りが決まったが、まだ正対に向き直していない
        self.alpha = 0.0
        self.alpha_trusted = False
        self.side_turn_ref = None         # 真横旋回の開始時の (機体x, 機体y, ヨー)
        self.side_marker_world = None     # 同時刻のマーカー位置（オドメトリ座標系）
        self.side_turn_checks = 0         # 真横旋回の測り直し回数（静止してから確認する）
        self.arrival = None               # 真横へ回る前の到達成績（旋回後は方位が無意味になる）
        self.done = False
        self.success = False
        self.reason = None

    def marker_optional(self):
        """いまマーカーが見えていなくてよい区間か。**見失い判定を止める側が使う**。

        真横へ90度回すと、マーカーは必ず視野(実機±46度)の外に出る。この区間は
        ヨー角だけで回しているので、見えなくても進行できるし、進行しなければならない。
        **区間の前後に入る静止待ちも含める**（静止中も見えていない。ここを外して
        いたせいで、静止に入った瞬間に「0.5s見失った」で落ちた）。
        """
        return (self.state == self.SIDE_TURN
                or (self.state == self.SETTLE and self.next_state == self.SIDE_TURN))

    def start(self, now):
        self._reset_state()
        self.events = [(now, self.state, '開始')]

    # ---- 法線（alpha）の推定 ----

    def update_alpha(self, gamma_obs, bearing, ambiguity):
        """曖昧性が十分なときだけ alpha を更新する（指数移動平均）。

        alpha = (法線の向き) - (マーカーからロボットへ向かう視線の向き)。
        その場旋回では bearing と法線が同じだけ回るので alpha は不変。
        """
        if gamma_obs is None or ambiguity < self.p.min_ambiguity:
            return
        a_obs = wrap(gamma_obs - (bearing + math.pi))
        if not self.alpha_trusted:
            self.alpha = a_obs
        else:
            self.alpha = wrap(self.alpha + self.p.normal_alpha * wrap(a_obs - self.alpha))
        self.alpha_trusted = True

    # ---- 幾何 ----

    def goal(self, mx, my):
        """マーカー方位と、ゴール点（法線上のstandoff点）を base_link 座標系で返す。

        法線は毎周期、その時点の視線方向から alpha を使って**再構成する**。
        保持したベクトルを回転させると、更新が止まっている間に劣化する。
        """
        bearing = math.atan2(my, mx)
        use_alpha = self.alpha_trusted and self.p.use_normal
        n_dir = bearing + math.pi + (self.alpha if use_alpha else 0.0)
        gx = mx + math.cos(n_dir) * self.p.standoff
        gy = my + math.sin(n_dir) * self.p.standoff
        return bearing, gx, gy

    # ---- 遷移 ----

    def _settle(self, now, next_state, reason):
        self.state = self.SETTLE
        self.next_state = next_state
        self.settle_until = now + self.p.settle_time
        self.events.append((now, next_state, reason))

    def _abort(self, now, reason, bearing):
        """打ち切りを決める。**ただし、その前に必ずマーカーへ向き直す。**

        ゴールに寄れないまま終わるとしても、機体はマーカーを正面に入れた状態で
        止まっていなければならない（次の手を打つのも、人が見て判断するのもそこから）。
        向き直さずに終えると、ゴールを向いたまま最大18度ずれて止まる（実装当初の不具合）。
        """
        self.abort_reason = reason
        if abs(bearing) > self.p.ang_tolerance:
            self.state = self.FINAL_TURN
            self.events.append((now, self.state, f'{reason} → 先に正対へ向き直す'))
        else:
            self._finish(now, False, reason)

    def _alpha_note(self):
        """報告文に付ける法線ずれ。**視線接近ではこれがそのまま残る**ので必ず出す。"""
        if not self.alpha_trusted:
            return ''
        return f'法線ずれ {math.degrees(self.alpha):+.1f}度、'

    def _finish(self, now, success, reason):
        self.state = self.DONE
        self.done = True
        self.success = success
        self.reason = reason
        self.events.append((now, self.DONE, reason))

    # ---- 本体 ----

    def turn_target(self, goal_bearing, bearing_cam):
        """その場旋回で回す角度[rad]。ゴール方位を、マーカーを視野に残せる範囲へ切り詰める。

        ゴール方位が範囲の外にあるときは「マーカーを画面に戻す向き」の値になる。
        絶対値が turn_tolerance 以下なら旋回の必要がない（＝旋回しても得がない）。
        """
        return clamp(goal_bearing,
                     bearing_cam - self.p.fov_budget, bearing_cam + self.p.fov_budget)

    def bearing_from_camera(self, mx, my):
        """カメラ座標系でのマーカー方位。**視野の判定はこれで行う**（base_linkではない）。"""
        return math.atan2(my - self.p.camera_y, mx - self.p.camera_x)

    def step(self, now, mx, my, gamma_obs, ambiguity, odom_obs=None):
        """1周期進める。

        `odom_obs` は機体の推測航法上の姿勢 `(x, y, yaw)`。**真横へ旋回するときだけ使う**
        （その区間はマーカーが視野から出るのでカメラで閉じられない）。
        絶対の基準は問わない（開始時からの差分しか使わない）。
        `final_heading='marker'`（既定）なら渡さなくてよい。

        **位置(x, y)も要る。** ヨーだけで回すと合わない。四足の「その場旋回」は
        その場ではなく、実測では90度回る間に機体が7cm前・8cm横へ動いた（2026-09-08、
        Gazebo）。マーカーまで0.58mなのでこれは方位10度に相当し、そのぶん真横から
        外れて止まっていた。旋回は正確（自己申告+86.0度／真値+86.0度）だったのに
        結果が合わなかった原因がこれ。位置が無い場合はヨーだけで回すが、
        その誤差は残る。
        """
        p = self.p
        bearing, _, _ = self.goal(mx, my)
        self.update_alpha(gamma_obs, bearing, ambiguity)
        bearing, gx, gy = self.goal(mx, my)          # alpha更新後の値で決める
        pos_err = math.hypot(gx, gy)
        goal_bearing = math.atan2(gy, gx)
        bearing_cam = self.bearing_from_camera(mx, my)
        dist = math.hypot(mx, my)
        if self.cycle_start_err is None:
            self.cycle_start_err = pos_err

        vx = wz = 0.0
        # 遷移が連鎖する（SETTLE明け→CHECK→TURN など）ので、同一周期内で数回まわす
        for _ in range(6):
            if self.state == self.DONE:
                break

            if self.state == self.SETTLE:
                if now < self.settle_until:
                    break
                self.state, self.next_state = self.next_state, None
                continue

            if self.state == self.CHECK:
                arrived = pos_err <= p.pos_tolerance and abs(bearing) <= p.ang_tolerance
                if self.abort_reason is not None and not arrived:
                    # 打ち切りは決まっている。正対に向き直したあとの値で報告する
                    self._finish(now, False,
                                 f'{self.abort_reason}'
                                 f'（位置誤差 {pos_err * 1000:.0f}mm、'
                                 f'方位 {math.degrees(bearing):+.1f}度、'
                                 f'{self._alpha_note()[:-1]}）')
                    continue
                # **打ち切りを決めたあとでも、向き直した結果が許容内なら到達である。**
                # 打ち切りの判定は「1周で縮まらない」等で下すが、その直後に入る
                # 正対旋回で方位が許容内に入ることがある。判定を向き直す前の値で
                # 確定させていたため、位置58mm・方位-4.0度（どちらも許容内）で
                # 止まっているのに失敗と報告していた（2026-09-08、Gazeboで発覚）。
                if arrived and self.abort_reason is not None:
                    self.abort_reason = None
                if arrived:
                    if p.final_heading != 'marker' and self.arrival is None:
                        # 位置は出来ている。ここから**向きだけ**を真横へ回す。
                        # 到達成績はこの時点の値で確定させる（旋回後は方位が無意味になる）
                        self.arrival = (pos_err, bearing, self.alpha, self.cycles + 1)
                        self._settle(now, self.SIDE_TURN,
                                     f'到達（位置誤差 {pos_err * 1000:.0f}mm）。'
                                     f'マーカーを{"右" if p.final_heading == "right" else "左"}'
                                     '真横に入れる旋回へ')
                        continue
                    self._finish(now, True,
                                 f'到達しました（位置誤差 {pos_err * 1000:.0f}mm、'
                                 f'方位 {math.degrees(bearing):+.1f}度、'
                                 f'{self._alpha_note()}{self.cycles + 1}周目）')
                elif self.cycles + 1 >= p.max_cycles:
                    self._abort(now, f'繰り返し上限 {p.max_cycles} 回で打ち切り', bearing)
                elif self.cycle_start_err - pos_err < p.min_progress:
                    # 折れ線で刻んでも届かない配置がある（法線から大きくずれた至近距離）。
                    # 視野の外へ動く必要があるので、粘っても近づけない。
                    self._abort(now, f'1周で {(self.cycle_start_err - pos_err) * 1000:.0f}mm しか'
                                     '縮まらないため打ち切り。'
                                     'マーカーを視野に残せる範囲では、ここより法線に寄れない',
                                bearing)
                else:
                    self.cycles += 1
                    self.cycle_start_err = pos_err
                    self.state = self.TURN_TO_GOAL
                    self.events.append((now, self.state,
                                        f'残差 {pos_err * 1000:.0f}mm。{self.cycles + 1}周目へ'))
                continue

            if self.state == self.TURN_TO_GOAL:
                if pos_err <= p.pos_tolerance:
                    self._settle(now, self.FINAL_TURN, 'すでにゴール圏内')
                    continue
                if abs(goal_bearing) > math.pi / 2:
                    # ゴールが背後。振り向くとマーカーを見失うので、向きを変えずに後退する
                    self.drive_dir = -1
                    self._settle(now, self.DRIVE, 'ゴールが背後にあるため後退する')
                    continue
                target = self.turn_target(goal_bearing, bearing_cam)
                if abs(target) <= max(p.turn_tolerance, p.turn_lead_angle):
                    self.drive_dir = 1
                    self._settle(now, self.DRIVE,
                                 f'ゴール方位 {math.degrees(goal_bearing):+.1f}度'
                                 f'（カメラ方位 {math.degrees(bearing_cam):+.1f}度）。直進に移る')
                    continue
                wz = math.copysign(clamp(p.k_yaw * abs(target), p.min_wz, p.max_wz), target)
                break

            if self.state == self.DRIVE:
                along = gx * self.drive_dir       # 進行方向に残っている距離
                if along <= p.stop_lead_distance:
                    if pos_err <= p.pos_tolerance:
                        self._settle(now, self.FINAL_TURN, 'ゴールに到達。正対に移る')
                    else:
                        self._settle(now, self.CHECK,
                                     f'直進を終了（横に {abs(gy) * 1000:.0f}mm 残）')
                    continue
                if self.drive_dir > 0 and pos_err > p.pos_tolerance:
                    if abs(bearing_cam) > p.drive_bearing_limit:
                        # 弦を進むとマーカーの方位は単調に増える。視野を使い切る前に切る
                        self._settle(now, self.CHECK,
                                     f'マーカーのカメラ方位が {math.degrees(bearing_cam):+.1f}度'
                                     'まで開いた（視野の限界に近い）')
                        continue
                    # ゴール方位がずれていても、**旋回で縮められるときだけ**切り直す。
                    # 視野の予算のためにあえてゴールから外している向きを「ずれている」と
                    # 判定して切ると、旋回と直進を往復して1mmも進まなくなる（実装当初の不具合）。
                    if abs(self.turn_target(goal_bearing, bearing_cam)) > p.redirect_tolerance:
                        self._settle(now, self.CHECK,
                                     f'ゴール方位が {math.degrees(goal_bearing):+.1f}度まで開き、'
                                     '旋回で縮められる')
                        continue
                vx = self.drive_dir * clamp(p.k_x * along, p.min_translation_speed, p.max_vx)
                break

            if self.state == self.SIDE_TURN:
                # **カメラでは追えない区間**。マーカーは視野の外に出るので、
                # ヨー角の差分だけで回す。到達判定は「回した角度」で行い、
                # マーカーの方位は一切見ない（見えていても信用しない）。
                if odom_obs is None or odom_obs[2] is None:
                    self._finish(now, False,
                                 f'final_heading={p.final_heading!r} だが推測航法の観測'
                                 '(odom_obs)が渡されていない。真横への旋回は'
                                 'カメラでは閉じられないので、オドメトリかIMUが要る')
                    continue
                ox, oy, oyaw = odom_obs
                if self.side_turn_ref is None:
                    self.side_turn_ref = (ox, oy, oyaw)
                    # 見えているうちにマーカーの位置を推測航法の座標系へ移して覚える。
                    # 以降はこれを現在の機体姿勢へ引き戻して方位を出す
                    if ox is not None and oy is not None:
                        c, sn = math.cos(oyaw), math.sin(oyaw)
                        self.side_marker_world = (ox + c * mx - sn * my,
                                                  oy + sn * mx + c * my)
                # マーカーを右真横に置く＝マーカー方位を -90度にする
                target_bearing = -p.side_turn_angle * (1.0 if p.final_heading == 'right'
                                                       else -1.0)
                if self.side_marker_world is not None and ox is not None:
                    # **位置の変化も入れて**、いまマーカーが何度に見えるはずかを出す
                    dx = self.side_marker_world[0] - ox
                    dy = self.side_marker_world[1] - oy
                    c, sn = math.cos(-oyaw), math.sin(-oyaw)
                    bx, by = c * dx - sn * dy, sn * dx + c * dy
                    bearing_pred = math.atan2(by, bx)
                    remain = wrap(bearing_pred - target_bearing)
                    turned = wrap(oyaw - self.side_turn_ref[2])
                else:
                    # 位置が無いのでヨーだけ。その場旋回でない分の誤差は残る
                    turned = wrap(oyaw - self.side_turn_ref[2])
                    remain = wrap(p.side_turn_angle
                                  * (1.0 if p.final_heading == 'right' else -1.0) - turned)
                # 静止して測り直した結果が許容内なら完了。**報告値は静止後の姿勢**
                if self.side_turn_checks > 0 and abs(remain) <= p.ang_tolerance:
                    pe, br, al, cy = self.arrival
                    self._finish(now, True,
                                 f'真横に構えました（旋回 {math.degrees(turned):+.1f}度、'
                                 f'マーカーの方位 '
                                 f'{math.degrees(target_bearing + remain):+.1f}度'
                                 f'／目標 {math.degrees(target_bearing):+.0f}度）。'
                                 '到達時の成績: '
                                 f'位置誤差 {pe * 1000:.0f}mm、方位 {math.degrees(br):+.1f}度、'
                                 f'{self._alpha_note()}{cy}周目')
                    continue
                # **ここは「目標角まで回す」旋回なので、惰性ぶん手前で指令を切る。**
                # 正対旋回(FINAL_TURN)が max(許容値, 惰性) で切るのは、目標が
                # 「マーカー方位ゼロ」でカメラが今の値を返し続けるからで、判定基準が違う。
                # こちらは絶対角が目標なので、turn_lead_angle だけ手前で切れば惰性で乗る。
                if abs(remain) <= p.turn_lead_angle:
                    if self.side_turn_checks >= 3:
                        pe, br, al, cy = self.arrival
                        self._finish(now, True,
                                     f'真横に構えました（旋回 {math.degrees(turned):+.1f}度、'
                                     f'マーカーの方位 '
                                     f'{math.degrees(target_bearing + remain):+.1f}度'
                                     f'／目標 {math.degrees(target_bearing):+.0f}度。'
                                     f'測り直し上限）。到達時の成績: '
                                     f'位置誤差 {pe * 1000:.0f}mm、'
                                     f'方位 {math.degrees(br):+.1f}度、'
                                     f'{self._alpha_note()}{cy}周目')
                        continue
                    self.side_turn_checks += 1
                    self._settle(now, self.SIDE_TURN, '静止して回した角度を測り直す')
                    continue
                wz = math.copysign(clamp(p.k_yaw * abs(remain), p.min_wz, p.max_wz), remain)
                break

            if self.state == self.FINAL_TURN:
                if abs(bearing) <= max(p.ang_tolerance, p.turn_lead_angle):
                    self._settle(now, self.CHECK, '正対した')
                    continue
                wz = math.copysign(clamp(p.k_yaw * abs(bearing), p.min_wz, p.max_wz), bearing)
                break

            raise RuntimeError(f'未知の状態: {self.state}')

        return Command(vx, wz, self.state, pos_err, bearing, bearing_cam, goal_bearing, dist,
                       self.alpha, self.alpha_trusted, self.cycles,
                       self.done, self.success, self.reason)
