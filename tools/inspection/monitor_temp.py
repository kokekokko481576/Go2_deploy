#!/usr/bin/env python3
"""Go2 の関節モーター温度とバッテリーを監視する（**購読のみ。機体へは何も送らない**）。2026-09-28

作った理由: 立ったままアームを横へ出した直後に赤点滅で転倒した。電源を入れ直した後でも
後脚の股関節（RR_hip / RL_hip）だけ70〜71℃で、他は33〜38℃だった。過熱保護が有力。
**保護が働く温度はまだ分かっていない**ので、既定のしきい値は仮。記録を見て決めること。

使い方（env_real.sh を source した端末で）:
  python3 ~/marker_detection/tools/monitor_temp.py
  python3 ~/marker_detection/tools/monitor_temp.py --warn 55 --alarm 65
  python3 ~/marker_detection/tools/monitor_temp.py --log ~/marker_detection/logs/temp_0928.tsv

表示（1秒ごと）:
  時刻 | 最高温度の関節 | 後脚股関節 RR/RL と上昇速度[℃/分] | 12関節の温度 | バッテリー残量・電圧
  しきい値を超えると行頭に [注意]/[危険] が付き、端末のベルが鳴る。
"""
import argparse
import collections
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowState

NAMES = ['FR_hip', 'FR_thigh', 'FR_calf', 'FL_hip', 'FL_thigh', 'FL_calf',
         'RR_hip', 'RR_thigh', 'RR_calf', 'RL_hip', 'RL_thigh', 'RL_calf']
WATCH = [6, 9]          # 後脚の股関節。転倒時に熱かったのはここ
TREND_WINDOW = 60.0     # 上昇速度を出す窓[s]


class TempMonitor(Node):
    def __init__(self, a):
        super().__init__('go2_temp_monitor')
        self.a = a
        self.last = None
        self.last_print = 0.0
        self.last_msg_time = None
        self.hist = collections.deque()     # (時刻, 12関節の温度)
        self.peak = [0] * 12
        self.log = None
        if a.log:
            self.log = open(a.log, 'a', buffering=1)
            if self.log.tell() == 0:
                self.log.write('unix_time\t' + '\t'.join(NAMES) + '\tsoc\tpower_v\n')
        self.create_subscription(LowState, '/lowstate', self.on_state, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)
        print(f'監視を開始します（/lowstate、購読のみ）。注意 {a.warn}℃ / 危険 {a.alarm}℃'
              f'（**仮の値**。保護が働く温度は未確認）', flush=True)

    def on_state(self, msg):
        self.last = msg
        self.last_msg_time = time.time()

    def trend(self, i, now):
        """直近 TREND_WINDOW 秒の上昇速度[℃/分]。データが短ければ None。"""
        if len(self.hist) < 2 or now - self.hist[0][0] < 10.0:
            return None
        t0, temps0 = self.hist[0]
        return (self.hist[-1][1][i] - temps0[i]) / (now - t0) * 60.0

    def tick(self):
        now = time.time()
        if self.last is None or now - self.last_msg_time > 3.0:
            print(time.strftime('%H:%M:%S') + '  /lowstate が届いていません'
                  '（env_real.sh を source したか、機体の電源と有線LANを確認）', flush=True)
            return
        m = self.last
        temps = [int(m.motor_state[i].temperature) for i in range(12)]
        self.hist.append((now, temps))
        while self.hist and now - self.hist[0][0] > TREND_WINDOW:
            self.hist.popleft()
        self.peak = [max(p, t) for p, t in zip(self.peak, temps)]
        hi = max(range(12), key=lambda i: temps[i])
        soc = m.bms_state.soc
        level = ('[危険] ' if temps[hi] >= self.a.alarm
                 else '[注意] ' if temps[hi] >= self.a.warn else '       ')
        watch = []
        for i in WATCH:
            tr = self.trend(i, now)
            watch.append(f'{NAMES[i]} {temps[i]:>3}℃'
                         + ('' if tr is None else f'({tr:+.1f}℃/分)'))
        line = (f'{level}{time.strftime("%H:%M:%S")}  最高 {NAMES[hi]} {temps[hi]}℃ | '
                + ' '.join(watch)
                + f' | 全 {" ".join(f"{t:>2}" for t in temps)}'
                + f' | 電池 {soc}% {m.power_v:.1f}V')
        if level.strip():
            line = '\a' + line
        print(line, flush=True)
        if self.log is not None:
            self.log.write(f'{now:.1f}\t' + '\t'.join(str(t) for t in temps)
                           + f'\t{soc}\t{m.power_v:.2f}\n')
        if soc <= self.a.low_soc:
            print(f'        電池残量 {soc}% が {self.a.low_soc}% 以下', flush=True)

    def summary(self):
        if any(self.peak):
            print('\n期間中の最高温度: ' + ' '.join(f'{n}:{p}' for n, p in zip(NAMES, self.peak)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--warn', type=int, default=60, help='注意を出す温度[℃]（仮）')
    ap.add_argument('--alarm', type=int, default=70, help='危険を出す温度[℃]（仮。転倒後の再起動時に70〜71℃）')
    ap.add_argument('--low-soc', type=int, default=20, help='電池残量の注意[%%]')
    ap.add_argument('--log', default='', help='1秒1行のTSVに追記する')
    a = ap.parse_args()
    rclpy.init()
    node = TempMonitor(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
