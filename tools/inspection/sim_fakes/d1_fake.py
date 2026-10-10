#!/usr/bin/env python3
"""D1 アームの代役（**sim の通し試験専用**）。~/d1_sdk/run.sh と同じ呼び方に答える。

  d1_fake.py set_joints a0 a1 a2 a3 a4 a5 a6   目標を記録する（関節ごとに最大 SPEED 度/秒で動いたことにする）
  d1_fake.py get_arm_joint_angle                実機と同じ書式 servoN_data:値 を数行出して終わる
  d1_fake.py arm_server                         ~/d1_sdk の arm_server と同じ行のやり取り(常駐)
                                                 D1_FAKE_DROP=N で最初のN回の set を捨てる(届かない再現)

状態は D1_FAKE_STATE（既定 ~/marker_detection/logs/sim/d1_fake_state.json）に置く。
初期値は収納姿勢（inspect_poses.json の stow ＋ グリッパ -12.6。10/1 D405 交換後の値、旧 D435i は 13.2）。
"""
import json
import os
import sys
import threading
import time
from pathlib import Path

SPEED = 40.0     # 度/秒（実機の1段あたりの所要から見た大まかな値）
STATE = Path(os.environ.get('D1_FAKE_STATE',
                            Path.home() / 'marker_detection' / 'logs' / 'sim' / 'd1_fake_state.json'))
STOW = [0.0, -89.3, 91.1, 0.0, 0.0, 0.0, -12.6]


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


def arm_server():
    """arm_server の代役。10Hz で A 行を出し、標準入力の set で目標を変える。"""
    lock = threading.Lock()
    out_lock = threading.Lock()
    drop = [int(os.environ.get('D1_FAKE_DROP', '0'))]
    t0 = time.time()
    done = threading.Event()

    def say(line):
        with out_lock:
            print(line, flush=True)

    def reader():
        seq = 100
        for line in sys.stdin:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'quit':
                break
            if parts[0] != 'set' or len(parts) != 8:
                say(f'E 読めない: {line.strip()}')
                continue
            seq += 1
            target = [float(v) for v in parts[1:]]
            say(f'S {seq} (fake) ' + ' '.join(f'{v:g}' for v in target))
            if drop[0] > 0:
                drop[0] -= 1      # 送ったが機体に届かなかったことにする
                continue
            with lock:
                st = load()
                STATE.parent.mkdir(parents=True, exist_ok=True)
                STATE.write_text(json.dumps({'from': now_angles(st), 'to': target, 't0': time.time()}))
        done.set()

    time.sleep(0.3)
    say('ready')
    threading.Thread(target=reader, daemon=True).start()
    while not done.is_set():
        with lock:
            a = now_angles(load())
        say(f'A {time.time() - t0:.3f} ' + ' '.join(f'{v:.2f}' for v in a))
        time.sleep(0.1)
    return 0


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
    if cmd == 'arm_server':
        return arm_server()
    print(f'd1_fake: 未対応のコマンド {cmd}', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
