#!/usr/bin/env python3
"""inspect_poses.json の展開経路(deploy)のうち、まとめて1回で送っても安全な段を探す（2026-10-07）。

**安全の基準: 動く関節それぞれが始点〜終点のどこにあっても干渉しないこと。**
サーボは同期して動くとは限らず、どの関節が先に着くか分からないので、
始点と終点を対角とする超直方体の中を格子で調べる（aim_pose のモデル・余裕 CLEARANCE 込み）。
貪欲に「今の段から、安全にまとめられる一番遠い段」へ飛ぶ。

    python3 merge_steps.py lie            # 結果を表示するだけ
    python3 merge_steps.py lie --write    # inspect_poses.json の deploy_fast に書く

伏せは床を trunk 原点の0.125m下とする（aim_pose --lying と同じ）。
"""
import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np

import aim_pose as ap

POSES = Path(__file__).resolve().parent / 'inspect_poses.json'


def penetration(p):
    return ap.collision([math.radians(p[0]), math.radians(p[1]), math.radians(p[2]), math.radians(p[4])])


def box_worst(a, b, n=7):
    """a〜b の超直方体の中で一番深い干渉量[m]。0なら安全。"""
    if abs(a[3] - b[3]) > 1e-6 or abs(a[5] - b[5]) > 1e-6:
        return float('inf')     # モデルは angle3/5 を扱わない
    axes = [np.linspace(a[i], b[i], n) if abs(a[i] - b[i]) > 1e-6 else [a[i]] for i in range(6)]
    return max(penetration(c) for c in itertools.product(*axes))


def merge(seq):
    out = [list(seq[0])]
    i = 0
    while i < len(seq) - 1:
        j = len(seq) - 1
        while j > i + 1 and box_worst(seq[i], seq[j]) > 0.0:
            j -= 1
        out.append(list(seq[j]))
        i = j
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('posture')
    p.add_argument("--write", action="store_true", help="deploy_fast を書き込む（JSON全体が整形し直されるので、手で入れるほうが差分は小さい）")
    a = p.parse_args()
    poses = json.loads(POSES.read_text())
    post = poses['postures'][a.posture]
    ap.FLOOR_Z = -0.125 if a.posture.startswith('lie') else ap.ars.FLOOR_Z
    seq = post['deploy']
    worst = max(box_worst(seq[i], seq[i + 1]) for i in range(len(seq) - 1))
    if worst > 0.0:
        raise SystemExit(f'元の経路の段単体でモデル上の干渉がある（{worst:.3f}m）。この基準ではまとめられない')
    fast = merge(seq)
    print(f'[{a.posture}] {len(seq) - 1}段 → {len(fast) - 1}段')
    for k in range(1, len(fast)):
        moved = [f'a{i}' for i in range(6) if abs(fast[k - 1][i] - fast[k][i]) > 1e-6]
        print(f'  {fast[k]}  動く関節: {",".join(moved)}')
    print(f'  (収納→撮影姿勢を1回で送った場合: 干渉 {box_worst(seq[0], seq[-1], 6):.3f}m)')
    if a.write:
        text = POSES.read_text()
        post['deploy_fast'] = fast
        post['_deploy_fast_説明'] = ('merge_steps.py が deploy から作った経路。各関節がどの順に着いても干渉しない段だけをまとめた。'
                                   '**実機の多関節同時指令は未試験**。inspect_run.py --fast-path のときだけ使う。deploy を直したら作り直すこと')
        POSES.write_text(json.dumps(poses, ensure_ascii=False, indent=2) + '\n')
        print(f'{POSES.name} に書きました（元の書式は整形し直されます）')


if __name__ == '__main__':
    main()
