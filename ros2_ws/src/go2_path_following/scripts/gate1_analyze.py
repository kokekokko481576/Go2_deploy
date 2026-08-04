#!/usr/bin/env python3
"""GATE1定量計測結果の解析 (Issue #36)。gate1_measure.pyが出すCSVを集計する。

使い方: ./gate1_analyze.py <gate1_*.csv | results_dir>
resultsディレクトリを渡した場合は最新のCSVを自動選択する。
"""
import csv
import statistics
import sys
from pathlib import Path


def load_rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) < 2:
        print(f'Usage: {sys.argv[0]} <gate1_*.csv | results_dir>')
        sys.exit(1)

    target = Path(sys.argv[1])
    if target.is_dir():
        csvs = sorted(target.glob('gate1_*.csv'))
        if not csvs:
            print(f'error: {target} に gate1_*.csv が見つかりません')
            sys.exit(1)
        target = csvs[-1]
        print(f'(最新のCSVを使用: {target.name})')

    rows = load_rows(target)
    if not rows:
        print('error: 試行データが空です')
        sys.exit(1)

    total = len(rows)
    successes = [r for r in rows if r['success'] == '1']
    failures = [r for r in rows if r['success'] == '0']
    n_success = len(successes)

    print('=== GATE1 定量ベースライン ===')
    print(f'試行数: {total}')
    print(f'到達成功率: {n_success}/{total} ({100.0 * n_success / total:.1f}%)')

    if successes:
        xy_errs = [float(r['xy_error_m']) for r in successes]
        yaw_errs = [float(r['yaw_error_deg']) for r in successes]
        print('\n-- 成功試行の誤差(ゴール姿勢誤差=xy、部材正対精度=yaw) --')
        print(f'xy誤差: 平均{statistics.mean(xy_errs):.3f}m'
              + (f' / 標準偏差{statistics.stdev(xy_errs):.3f}m' if len(xy_errs) > 1 else '')
              + f' / 最大{max(xy_errs):.3f}m')
        print(f'yaw誤差: 平均{statistics.mean(yaw_errs):.2f}deg'
              + (f' / 標準偏差{statistics.stdev(yaw_errs):.2f}deg' if len(yaw_errs) > 1 else '')
              + f' / 最大{max(yaw_errs):.2f}deg')

    if failures:
        no_tf = [r for r in failures if r['final_x'] == '']
        timeouts = [r for r in failures if r['final_x'] != '']
        print(f'\n-- 失敗{len(failures)}件の内訳 --')
        if no_tf:
            print(f'TF取得不可: {len(no_tf)}件(EKF/AMCL未起動・TF remap忘れの疑い)')
        if timeouts:
            xy_errs_fail = [float(r['xy_error_m']) for r in timeouts]
            print(f'タイムアウト(到達判定に届かず): {len(timeouts)}件'
                  f' / 時点xy誤差: 平均{statistics.mean(xy_errs_fail):.3f}m'
                  f' / 最大{max(xy_errs_fail):.3f}m')

    # Issue #43: 真値(gt_*列)があれば推定ベースとの食い違いを報告する
    gt_rows = [r for r in rows if r.get('gt_success', '') != '']
    if gt_rows:
        gt_success_rows = [r for r in gt_rows if r['gt_success'] == '1']
        mismatches = [r for r in gt_rows if r['gt_success'] != r['success']]
        print(f'\n-- 真値(Gazebo物理)との比較({len(gt_rows)}/{total}試行で取得) --')
        print(f'真値ベース到達成功率: {len(gt_success_rows)}/{len(gt_rows)} '
              f'({100.0 * len(gt_success_rows) / len(gt_rows):.1f}%)')
        if mismatches:
            print(f'推定と真値で判定が食い違った試行: {len(mismatches)}件'
                  '(推定=成功/真値=未到達 の偽陽性、またはその逆)')
            for r in mismatches:
                print(f"  trial {r['trial']}: 推定success={r['success']} "
                      f"gt_success={r['gt_success']} "
                      f"(gt_xy_err={r['gt_xy_error_m']}m, gt_yaw_err={r['gt_yaw_error_deg']}deg)")
        else:
            print('推定と真値の判定が食い違った試行: 0件')
    else:
        print('\n(真値データ無し。start_ground_truth.shを起動していない計測と思われます)')


if __name__ == '__main__':
    main()
