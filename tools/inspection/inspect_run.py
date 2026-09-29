#!/usr/bin/env python3
"""ビード検査の一連の動作を1本で進める（2026-09-28）。

    接近（リモコン or マーカー）→ [伏せる] → アームを撮影姿勢へ → 撮影 → アームを収納 → [起立]

--posture lie（2026-09-29 からデモの本線）: 到着したら StopMove → StandDown で伏せ、**体の高さが下がったのを確かめてから**
アームを出す。撮影・収納は伏せたまま行い、最後に StandUp → BalanceStand で起立して終わる。
立っている時間が短くなるので、後脚股関節の過熱（2026-09-28 の転倒）にも効く。

**機体を動かすスクリプトである。** 区切りごとに「Enterで次へ」と止まる（`--auto` で止めない）。
非常停止（estop.sh・物理リモコン）はこれまでどおり効く。このスクリプトを Ctrl+C で止めても
アームはその場に残るので、収納は手で行うこと（止めた位置からの収納コマンドを表示する）。

前提:
  - 端末ごとに `source ~/marker_detection/tools/env_real.sh`
  - マーカー接近のときは別端末で approach_real.launch.py を起動しておく（enable=False で待機している状態）
  - アームは収納姿勢（inspect_poses.json の stow）から始める
  - アームの姿勢は inspect_poses.json。**経路は干渉を確認したものだけ入れること**

止める条件（止めたら、その場から収納するか聞く）:
  - 後脚の股関節の温度が --hip-alarm 以上（2026-09-28 転倒時に70〜71℃）
  - アームが指令した角度に届かない（--tolerance 以上ずれたまま）
  - マーカー接近が「到達」以外で終わった

使い方:
  python3 inspect_run.py --approach manual --posture stand      # リモコンで寄せて、立ったまま
  python3 inspect_run.py --approach marker --posture lie        # マーカーで接近し、伏せてから
  python3 inspect_run.py --approach manual --posture lie --dry-run   # 何も動かさず流れだけ確認
"""
import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
D1 = Path.home() / 'd1_sdk' / 'run.sh'
RS_SNAP = HERE / 'rs_snap.sh'
LOG_DIR = HERE.parent / 'logs'
NAMES = ['FR_hip', 'FR_thigh', 'FR_calf', 'FL_hip', 'FL_thigh', 'FL_calf',
         'RR_hip', 'RR_thigh', 'RR_calf', 'RL_hip', 'RL_thigh', 'RL_calf']
HIPS = [6, 9]
API = {'STANDUP': 1004, 'STANDDOWN': 1005, 'BALANCESTAND': 1002, 'STOPMOVE': 1003}


class Abort(Exception):
    pass


class Robot:
    """ROS まわり（温度・マーカー・接近ノード・姿勢指令）。--dry-run では作らない。"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rcl_interfaces.msg import Log
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool
        from unitree_go.msg import LowState, SportModeState
        from unitree_api.msg import Request
        self.rclpy, self.Bool, self.Request = rclpy, Bool, Request
        rclpy.init()
        self.node = Node('inspect_run')
        self.temps = None
        self.soc = None
        self.lowstate_t = None
        self.pose_t = None
        self.approach_result = None      # (成功?, 文面)
        self.approach_state = None
        self.body_h = None            # /sportmodestate の body_height（伏せ・起立の確認に使う）
        self.sport_t = None
        n = self.node
        n.create_subscription(LowState, '/lowstate', self._on_low, qos_profile_sensor_data)
        n.create_subscription(SportModeState, '/sportmodestate', self._on_sport, qos_profile_sensor_data)
        n.create_subscription(PoseStamped, '/marker_detector_node/pose', self._on_pose, 10)
        n.create_subscription(Log, '/rosout', self._on_log, 50)
        from std_msgs.msg import String
        n.create_subscription(String, '/marker_approach_node/state', self._on_state, 10)
        self.enable_pub = n.create_publisher(Bool, '/marker_approach_node/enable', 10)
        self.sport_pub = n.create_publisher(Request, '/api/sport/request', 10)
        from rclpy.executors import SingleThreadedExecutor
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    def _on_low(self, m):
        self.temps = [int(m.motor_state[i].temperature) for i in range(12)]
        self.soc = m.bms_state.soc
        self.lowstate_t = time.time()

    def _on_sport(self, m):
        self.body_h = float(m.body_height)
        self.sport_t = time.time()

    def body_height(self):
        """直近の body_height[m]。2秒以上届いていなければ None。"""
        if self.sport_t is None or time.time() - self.sport_t > 2.0:
            return None
        return self.body_h

    def _on_pose(self, _m):
        self.pose_t = time.time()

    def _on_state(self, m):
        self.approach_state = m.data

    def _on_log(self, m):
        if m.name.endswith('marker_approach_node') and '停止しました' in m.msg:
            self.approach_result = ('到達しました' in m.msg, m.msg)

    def wait_lowstate(self, timeout=10.0):
        t0 = time.time()
        while self.lowstate_t is None or time.time() - self.lowstate_t > 2.0:
            if time.time() - t0 > timeout:
                raise Abort('/lowstate が届かない（env_real.sh・機体の電源・有線LANを確認）')
            time.sleep(0.2)

    def hip_temps(self):
        self.wait_lowstate()
        return [self.temps[i] for i in HIPS]

    def marker_visible(self):
        return self.pose_t is not None and time.time() - self.pose_t < 1.0

    def enable_approach(self, on):
        self.approach_result = None
        for _ in range(3):
            self.enable_pub.publish(self.Bool(data=on))
            time.sleep(0.1)

    def sport(self, name):
        req = self.Request()
        req.header.identity.api_id = API[name]
        for _ in range(3):
            self.sport_pub.publish(req)
            time.sleep(0.1)

    def close(self):
        # 受信スレッドを先に止めてから片付ける（逆順だとプロセスが落ちる）
        try:
            self.ex.shutdown(timeout_sec=1.0)
            self.th.join(timeout=2.0)
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


class Runner:
    def __init__(self, a, poses):
        self.a = a
        self.poses = poses
        self.grip = poses['gripper']
        self.tol = a.tolerance if a.tolerance is not None else poses['tolerance_deg']
        self.cur_index = None       # いま deploy の何番目にいるか（収納の起点）
        self.bracket_dirty = False  # 撮影のために関節を振っていて、基準の姿勢から外れている
        self.robot = None if a.dry_run else Robot()
        LOG_DIR.mkdir(exist_ok=True)
        self.logf = open(LOG_DIR / time.strftime('inspect_%m%d_%H%M%S.log'), 'w', buffering=1)

    # ---- 表示・確認 ----
    def say(self, s):
        line = time.strftime('%H:%M:%S ') + s
        print(line, flush=True)
        self.logf.write(line + '\n')

    def confirm(self, what, force=False):
        """区切りで止める。--auto でも force=True（人が作業する区切り）は止める。"""
        if self.a.auto and not force:
            self.say(f'[自動] {what}')
            return
        try:
            ans = input(f'\n>>> {what}\n    Enter で実行 / q で中止: ')
        except EOFError:
            ans = 'q'
        if ans.strip().lower() == 'q':
            raise Abort('中止が選ばれました')

    # ---- 温度 ----
    def check_temp(self, where):
        if self.robot is None:
            return
        rr, rl = self.robot.hip_temps()
        msg = f'後脚股関節 RR {rr}℃ / RL {rl}℃、電池 {self.robot.soc}%'
        if max(rr, rl) >= self.a.hip_alarm:
            raise Abort(f'{where}: {msg} がしきい値 {self.a.hip_alarm}℃ 以上')
        if max(rr, rl) >= self.a.hip_warn:
            self.say(f'  [注意] {msg}（注意 {self.a.hip_warn}℃）')
        else:
            self.say(f'  {msg}')

    # ---- アーム ----
    def read_arm(self):
        """angle0〜6 を返す。get_arm_joint_angle を数秒だけ走らせて最後の行を読む。"""
        if self.a.dry_run:
            return None
        try:
            out = subprocess.run(['timeout', '4', str(D1), 'get_arm_joint_angle'],
                                 capture_output=True, text=True).stdout
        except Exception as e:
            raise Abort(f'アームの角度を読めない: {e}')
        rows = [l for l in out.splitlines() if 'servo0_data' in l]
        if not rows:
            raise Abort('アームの角度を読めない（D1の電源・通信を確認）')
        vals = re.findall(r'servo(\d)_data:\s*(-?[\d.]+)', rows[-1])
        return [float(v) for _, v in sorted(vals, key=lambda x: int(x[0]))]

    def arm_to(self, pose, label):
        cmd = [str(D1), 'set_joints'] + [f'{v:g}' for v in pose] + [f'{self.grip:g}']
        self.say(f'  アーム {label}: set_joints {" ".join(cmd[2:])}')
        if self.a.dry_run:
            return
        subprocess.run(cmd, capture_output=True, text=True)
        t0 = time.time()
        while True:
            now = self.read_arm()
            err = max(abs(now[i] - pose[i]) for i in range(6))
            if err <= self.tol:
                self.say(f'    到達（最大ずれ {err:.1f}度）')
                return
            if time.time() - t0 > self.a.arm_timeout:
                raise Abort(f'アームが指令に届かない（最大ずれ {err:.1f}度、実測 '
                            f'{" ".join(f"{v:.1f}" for v in now[:6])}）')
            time.sleep(0.5)

    def check_arm_start(self):
        if self.a.dry_run:
            return
        now = self.read_arm()
        stow = self.poses['stow']
        err = max(abs(now[i] - stow[i]) for i in range(3))
        self.say(f'  アームの現在角 {" ".join(f"{v:.1f}" for v in now)}')
        if abs(now[6] - self.grip) > 2.0:
            raise Abort(f'グリッパ(angle6)が {now[6]:.1f} で、設定 {self.grip} と違う。'
                        '設定のまま送るとグリッパが動いてカメラが落ちるので止める')
        if err <= 5.0 and max(abs(now[i]) for i in (3, 4, 5)) <= 3.0:
            return
        # 肩と肘が収納のままなら、根元の旋回と手首だけの小さな動きで収納姿勢に合わせられる。
        # 転倒・電源の入れ直しのあと angle0 が 76.2（収納 68.7）になっていた（2026-09-28）
        shoulder_elbow = max(abs(now[i] - stow[i]) for i in (1, 2))
        if shoulder_elbow > 5.0:
            raise Abort('アームが収納姿勢にない（肩・肘のずれ '
                        f'{shoulder_elbow:.1f}度）。手で収納してから始めること')
        self.confirm(f'アームを収納姿勢 {stow} に合わせます'
                     '（肩・肘は収納のまま。根元の旋回と手首だけが動く）', force=True)
        # 手首を先に1軸ずつ0へ（小さい動き）、最後に根元の旋回
        p = list(now[:6])
        for i in (3, 4, 5):
            if abs(p[i]) > 0.5:
                p[i] = 0.0
                self.arm_to(p, f'手首{i}を0へ')
        p[0] = stow[0]
        self.arm_to(p, '根元の旋回を収納へ')
        self.arm_to(stow, '収納姿勢')

    def deploy(self, seq):
        for i, p in enumerate(seq):
            if i == 0:
                self.cur_index = 0
                continue
            self.check_temp(f'アーム {i}/{len(seq) - 1} の前')
            self.confirm(f'アームを {i}/{len(seq) - 1} 段目へ: {p}')
            self.arm_to(p, f'{i}/{len(seq) - 1}')
            self.cur_index = i

    def snap(self, name):
        """1枚撮る。背中の Jetson で撮るので、**Jetson が再起動中だと黙って失敗する**
        （2026-09-28 15:28、アームを出した状態で Jetson が再起動し、写真が残らなかった）。保存を確かめる。"""
        while not self.a.dry_run:
            env = dict(os.environ, RS_DEPTH=self.a.depth) if self.a.depth else None
            r = subprocess.run(['timeout', '60', str(RS_SNAP), name], env=env)
            png = LOG_DIR / 'rs' / f'{name}_color.png'
            if r.returncode == 0 and png.exists():
                break
            self.say(f'  [失敗] 撮影できなかった（終了コード {r.returncode}）。'
                     'Jetson(192.168.123.18) に ssh できるか確認')
            try:
                ans = input('    r で撮り直す / それ以外で撮らずに進む: ')
            except EOFError:
                ans = ''
            if ans.strip().lower() != 'r':
                break
        self.say(f'  撮影: logs/rs/{name}_color.png')

    def stow(self, seq):
        if self.cur_index is None or self.cur_index == 0:
            return
        if getattr(self, 'bracket_dirty', False):
            # 撮影のために関節を振った途中で止まった。まず基準の姿勢へ戻す（振った関節と次の段の関節を同時に動かさない）
            self.confirm(f'撮影で振った関節を基準へ戻す {seq[self.cur_index]}')
            self.arm_to(seq[self.cur_index], '撮影の基準姿勢へ戻す')
            self.bracket_dirty = False
        back = list(range(self.cur_index - 1, -1, -1))
        self.say(f'収納します（{len(back)}段。展開の経路を逆にたどる）')
        for i in back:
            self.confirm(f'収納 {seq[i]}')
            self.arm_to(seq[i], f'収納 {len(back) - back.index(i)}/{len(back)}')
            self.cur_index = i

    def print_manual_stow(self, seq):
        if self.cur_index is None or self.cur_index == 0:
            return
        print('\n手で収納するときは、上から順に1行ずつ:')
        for i in range(self.cur_index - 1, -1, -1):
            print(f'  {D1} set_joints {" ".join(f"{v:g}" for v in seq[i])} {self.grip:g}')

    # ---- 伏せ・起立 ----
    def _wait_height(self, cond, timeout):
        """body_height が cond を満たすまで待つ。満たせば (True, 値)、時間切れなら (False, 最後の値)。"""
        t0 = time.time()
        h = None
        while time.time() - t0 < timeout:
            h = self.robot.body_height()
            if h is not None and cond(h):
                return True, h
            time.sleep(0.1)
        return False, h

    def _wait_settled(self, timeout, still=0.005, span=0.6):
        """体の高さが落ち着く（直近 span 秒の変化が still m 未満）まで待つ。落ち着いた高さを返す。

        「下がり始めた」で先へ進むと、伏せ切る前にアームを出してしまう（sim で 0.23m の時点で出していた）。
        """
        t0 = time.time()
        hist = []
        while time.time() - t0 < timeout:
            h = self.robot.body_height()
            now = time.time()
            if h is not None:
                hist.append((now, h))
                hist = [x for x in hist if now - x[0] <= span]
                if now - hist[0][0] >= span * 0.8 and max(x[1] for x in hist) - min(x[1] for x in hist) < still:
                    return True, h
            time.sleep(0.1)
        return False, (hist[-1][1] if hist else None)

    def _ask_eyes(self, what):
        """高さで確認できなかったとき、人の目で確かめてもらう（--auto でも必ず止める）。"""
        try:
            ans = input(f'\n>>> {what}を体の高さで確認できませんでした。目で見て{what}ていれば y / それ以外で中止: ')
        except EOFError:
            ans = ''
        if ans.strip().lower() != 'y':
            raise Abort(f'{what}たことを確認できないので止める')

    def lie_down(self):
        """伏せる。**伏せたことを確かめてからアームへ進む**（立ったまま伏せ用の姿勢を出さないため）。

        body_height が実機で伏せ・起立に追従するかは未確認（2026-09-29 追加）。確認できなければ人に聞く。
        """
        h0 = self.robot.body_height()
        self.robot.sport('STOPMOVE')
        time.sleep(0.5)
        self.robot.sport('STANDDOWN')
        ok, h = self._wait_height(lambda v: h0 is not None and v < h0 - 0.08, self.a.posture_wait + 4.0)
        if ok:
            ok, h = self._wait_settled(self.a.posture_wait + 4.0)   # 下がり切るまで待つ
        self.say(f'  体の高さ: 伏せる前 {h0 if h0 is None else round(h0, 3)} m → 伏せた後 {h if h is None else round(h, 3)} m')
        if not ok:
            self._ask_eyes('伏せ')

    def stand_up(self):
        h0 = self.robot.body_height()
        self.robot.sport('STANDUP')
        ok, h = self._wait_height(lambda v: h0 is not None and v > h0 + 0.08, self.a.posture_wait + 4.0)
        if ok:
            ok, h = self._wait_settled(self.a.posture_wait + 4.0)   # 立ち上がり切るまで待つ
        self.say(f'  体の高さ: 起立前 {h0 if h0 is None else round(h0, 3)} m → 起立後 {h if h is None else round(h, 3)} m')
        if not ok:
            self._ask_eyes('起立し')
        self.robot.sport('BALANCESTAND')   # これを送らないと、次に歩く指令を受け付けない

    # ---- 本体 ----
    def run(self):
        a = self.a
        post = self.poses['postures'][a.posture]
        seq = post['deploy']
        self.say(f'開始: 接近={a.approach} 姿勢={a.posture} '
                 f'{"[dry-run 何も動かさない]" if a.dry_run else ""}{"[auto]" if a.auto else ""}')
        if post.get('provisional'):
            self.say(f'  [注意] {a.posture} の姿勢は**仮**です: {post.get("_説明", "")}')
        try:
            # 0. 開始前の確認
            self.check_temp('開始前')
            self.check_arm_start()

            # 1. 接近
            if a.approach == 'manual':
                self.confirm('リモコンで撮影位置へ寄せてください（部材から20cm、右前足の軸を'
                             '部材の左前の角に）。止めたら Enter', force=True)
            else:
                if self.robot is not None and not self.robot.marker_visible():
                    raise Abort('マーカーが見えていない（/marker_detector_node/pose が来ない）。'
                                'approach_real.launch.py の起動とマーカーの位置を確認')
                self.confirm('マーカー接近を開始します（機体が歩きます）')
                if self.robot is not None:
                    self.robot.enable_approach(True)
                    t0 = time.time()
                    last = None
                    while self.robot.approach_result is None:
                        if self.robot.approach_state != last:
                            last = self.robot.approach_state
                            self.say(f'  接近: {last}')
                        if time.time() - t0 > a.approach_timeout:
                            self.robot.enable_approach(False)
                            raise Abort(f'接近が {a.approach_timeout}s で終わらない')
                        time.sleep(0.2)
                    ok, text = self.robot.approach_result
                    self.say(f'  接近の結果: {text}')
                    if not ok:
                        raise Abort('接近が「到達」以外で終わった')
                else:
                    self.say('  [dry-run] 接近は到達したものとする')

            # 2. 伏せる
            if a.posture == 'lie':
                self.check_temp('伏せる前')
                self.confirm('伏せます（StopMove → StandDown）')
                if self.robot is not None:
                    self.lie_down()
                self.say('  伏せました')

            # 3. アームを出す
            self.deploy(seq)

            # 4. 撮影（姿勢に bracket があれば、関節を振って複数枚）
            name = time.strftime(f'{a.name}_%m%d_%H%M%S')
            bracket = post.get('bracket') if not a.no_bracket else None
            if bracket:
                j = bracket['joint']
                self.confirm(f'撮影します（angle{j} を {bracket["angles"]} と振って{len(bracket["angles"])}枚）')
                for k, ang in enumerate(bracket['angles']):
                    self.check_temp(f'撮影 {k + 1}/{len(bracket["angles"])} の前')
                    p = list(seq[-1])
                    p[j] = ang
                    self.bracket_dirty = (ang != seq[-1][j])
                    self.arm_to(p, f'撮影 {k + 1}/{len(bracket["angles"])}（angle{j}={ang:g}）')
                    self.snap(f'{name}_{k + 1}_a{j}_{ang:g}')
                # 基準の姿勢へ戻してから収納へ（収納は展開の経路を逆にたどるので、出発点を合わせる）
                self.arm_to(seq[-1], '撮影の基準姿勢へ戻す')
                self.bracket_dirty = False
            else:
                self.confirm(f'撮影します（{name}）')
                self.snap(name)

            # 5. 収納（動画用に、撮影姿勢を少し見せてから）
            if a.hold > 0:
                self.say(f'  撮影姿勢のまま {a.hold:g} 秒待ちます')
                time.sleep(a.hold)
            self.confirm('アームを収納します', force=not a.auto)
            self.stow(seq)

            # 6. 起立
            if a.posture == 'lie':
                self.confirm('起立します（StandUp → BalanceStand）', force=not a.auto)
                if self.robot is not None:
                    self.stand_up()
                self.say('  起立しました')
            self.check_temp('終了時')
            self.say('完了')
            return 0
        except Abort as e:
            self.say(f'[停止] {e}')
            if self.robot is not None:
                self.robot.enable_approach(False)
            if self.cur_index:
                try:
                    ans = input('\nアームをこの位置から収納しますか？ y で収納 / それ以外で終了: ')
                except EOFError:
                    ans = ''
                if ans.strip().lower() == 'y':
                    try:
                        self.a.auto = False
                        self.stow(seq)
                    except Abort as e2:
                        self.say(f'[停止] 収納中: {e2}')
                        self.print_manual_stow(seq)
                else:
                    self.print_manual_stow(seq)
            return 1
        except KeyboardInterrupt:
            self.say('[中断] Ctrl+C')
            if self.robot is not None:
                self.robot.enable_approach(False)
            self.print_manual_stow(seq)
            return 130
        finally:
            if self.robot is not None:
                self.robot.close()


def main():
    global D1, RS_SNAP
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--approach', choices=['manual', 'marker'], required=True,
                    help='manual=リモコンで寄せてEnter / marker=approach_real.launch.py で自動接近')
    ap.add_argument('--posture', choices=['stand', 'lie'], required=True,
                    help='stand=立ったまま / lie=止まってから伏せてアーム')
    ap.add_argument('--auto', action='store_true', help='区切りで止めない（人の作業が要る区切りだけは止める）')
    ap.add_argument('--dry-run', action='store_true', help='何も動かさず、送る指令を表示するだけ')
    ap.add_argument('--poses', default=str(HERE / 'inspect_poses.json'))
    ap.add_argument('--name', default='inspect', help='撮影ファイル名の頭')
    ap.add_argument('--hip-warn', type=int, default=55)
    ap.add_argument('--hip-alarm', type=int, default=65,
                    help='後脚股関節がこの温度[℃]以上なら止める（仮。転倒時は70〜71℃）')
    ap.add_argument('--tolerance', type=float, default=None, help='アーム到達の許容[度]')
    ap.add_argument('--arm-timeout', type=float, default=10.0)
    ap.add_argument('--approach-timeout', type=float, default=120.0)
    ap.add_argument('--d1-run', default=str(D1),
                    help='D1 SDK の run.sh（sim の通し試験では tools/sim_fakes/d1_fake.py を渡す）')
    ap.add_argument('--rs-snap', default=str(RS_SNAP),
                    help='撮影スクリプト（sim の通し試験では tools/sim_fakes/rs_snap_fake.sh を渡す）')
    ap.add_argument('--no-bracket', action='store_true',
                    help='姿勢に bracket（関節を振って複数枚撮る）があっても、基準の1枚だけにする')
    ap.add_argument('--depth', default='',
                    help='深度の撮影解像度（例 848x480）。省略時は 1280x720（最短測距 約0.28m）')
    ap.add_argument('--hold', type=float, default=0.0, help='撮影のあと、収納する前に待つ秒数（動画用）')
    ap.add_argument('--posture-wait', type=float, default=4.0, help='伏せ・起立の待ち[s]')
    a = ap.parse_args()
    D1, RS_SNAP = Path(a.d1_run), Path(a.rs_snap)
    poses = json.load(open(a.poses))
    return Runner(a, poses).run()


if __name__ == '__main__':
    sys.exit(main())
