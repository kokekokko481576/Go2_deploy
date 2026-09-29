#!/usr/bin/env python3
"""D1 アームの代役（**sim の通し試験専用**）。~/d1_sdk/run.sh と同じ呼び方に答える。

  d1_fake.py set_joints a0 a1 a2 a3 a4 a5 a6   目標を記録する（関節ごとに最大 SPEED 度/秒で動いたことにする）
  d1_fake.py get_arm_joint_angle                実機と同じ書式 servoN_data:値 を数行出して終わる

状態は D1_FAKE_STATE（既定 ~/marker_detection/logs/sim/d1_fake_state.json）に置く。
初期値は収納姿勢（inspect_poses.json の stow ＋ グリッパ 13.2）。
"""
import json
import os
import sys
import time
from pathlib import Path

SPEED = 40.0     # 度/秒（実機の1段あたりの所要から見た大まかな値）
STATE = Path(os.environ.get('D1_FAKE_STATE',
                            Path.home() / 'marker_detection' / 'logs' / 'sim' / 'd1_fake_state.json'))
STOW = [68.7, -89.3, 91.1, 0.0, 0.0, 0.0, 13.2]


def load():
    if STATE.exists():
        return json.loads(STATE.read_text())
    return {'from': STOW, 'to': STOW, 't0': 0.0}


def now_angles(st):
    t = time.time() - st['t0']
    out = []
    for a, b in zip(st['from'], st['to']):
        d = b - a
        step = SPEED * t
        out.append(b if abs(d) <= step else a + step * (1 if d > 0 else -1))
    return out


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    st = load()
    if cmd == 'set_joints':
        target = [float(v) for v in sys.argv[2:9]]
        cur = now_angles(st)
        STATE.parent.mkdir(parents=True, exist_ok=True)
        STATE.write_text(json.dumps({'from': cur, 'to': target, 't0': time.time()}))
        print('送信: (fake) ' + ' '.join(f'{v:g}' for v in target))
        return 0
    if cmd == 'get_arm_joint_angle':
        for _ in range(3):
            a = now_angles(load())
            print(', '.join(f'servo{i}_data:{v:.1f}' for i, v in enumerate(a)), flush=True)
            time.sleep(0.1)
        return 0
    print(f'd1_fake: 未対応のコマンド {cmd}', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
