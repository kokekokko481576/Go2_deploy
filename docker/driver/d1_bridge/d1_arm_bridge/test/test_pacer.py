"""送信間隔の下限を守る待ち行列の検証。指令を捨てず、順番どおりに送ることを押さえる。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from d1_arm_bridge.pacer import CommandPacer  # noqa: E402


def test_最初の指令はすぐ送れる():
    p = CommandPacer(25.0, 10)
    p.push('a')
    assert p.pop_ready(0.0) == 'a'


def test_下限に満たない指令は捨てずに下限が明けたら順番どおり送る():
    # d1_arm_demo が 4秒おきに送ってきても、どれも抜けない
    p = CommandPacer(25.0, 10)
    sent = []
    for t in range(0, 200):
        if t in (0, 4, 8, 12):
            p.push(f'wp{t}')
        item = p.pop_ready(float(t))
        if item is not None:
            sent.append((t, item))
    assert [item for _, item in sent] == ['wp0', 'wp4', 'wp8', 'wp12']
    times = [t for t, _ in sent]
    assert all(b - a >= 25.0 for a, b in zip(times, times[1:]))


def test_満杯なら積まない():
    p = CommandPacer(25.0, 2)
    assert p.push('a') and p.push('b')
    assert not p.push('c')
    assert list(p.pending) == ['a', 'b']


def test_ゼロ姿勢は下限を無視し待ち行列を捨てる():
    p = CommandPacer(25.0, 10)
    p.push('a')
    p.pop_ready(0.0)
    p.push('b')
    p.push('c')
    assert p.bypass(2.0) == 2
    assert p.pop_ready(3.0) is None
    # 復帰後の次の指令は、復帰から下限が明けるまで待つ
    p.push('d')
    assert p.pop_ready(26.0) is None
    assert p.pop_ready(27.0) == 'd'
