#!/usr/bin/env python3
"""slam_toolboxの地図(pose graph全体)を定期的にチェックポイント保存する。

2026-11-16、長時間歩行中に自己位置推定が突然壊れ、そのまま戻らなくなる現象が
繰り返し起きている(エラーログなし、原因不明のまま)。「壊れることを防ぐ」のではなく
「壊れたら直前の良好な状態まで巻き戻せる」という実用的な保険として、
slam_toolbox標準のserialize_map/deserialize_mapサービスを使う。

使い方:
  python3 slam_checkpoint.py --interval 60 --out-dir ~/ros2_ws/checkpoints
  (観測のみ。slam_toolboxへの書き込みはserialize_mapの呼び出しのみで、
   地図や走行には影響しない)

壊れた後の復元(別ターミナルで):
  ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
    "{filename: '<保存先>/checkpoint_XXXXXX', match_type: 1}"
  (match_type=1: START_AT_FIRST_NODE。壊れる前のpose graphの最初のノード基準で復元し、
   そこからslam_toolboxがまた地図を伸ばせる状態に戻る)

**既知の罠(2026-11-16実機録画データで確認)**: スキャンがほぼ0件の「空に近い」状態で保存した
チェックポイントをdeserializeすると、slam_toolbox本体がエラーログも無くSIGSEGVで
サイレントクラッシュする(upstream GitHub Issue #902で報告されている既知の挙動、
Humble版での修正有無は未確認)。これを踏まえ、起動直後の数十秒はチェックポイントを
取らないよう`--warmup`で猶予を設けている。十分なスキャンが蓄積された状態のチェックポイント
であれば、スキャン処理を止めずにdeserializeしてもクラッシュしないことを確認済み
(`pause_new_measurements`での一時停止は不要だった)。
"""
import argparse
import os
import time

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph


class SlamCheckpoint(Node):
    def __init__(self, a):
        super().__init__('slam_checkpoint')
        self.a = a
        os.makedirs(a.out_dir, exist_ok=True)
        self.cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.count = 0
        self.start_time = time.monotonic()
        self.create_timer(a.interval, self.tick)
        print(f'チェックポイントを{a.interval}秒おきに{a.out_dir}へ保存します '
              f'(起動後{a.warmup}秒は空セッション対策でスキップ)', flush=True)

    def tick(self):
        elapsed = time.monotonic() - self.start_time
        if elapsed < self.a.warmup:
            print(f'{time.strftime("%H:%M:%S")}  warmup中({elapsed:.0f}/{self.a.warmup:.0f}s)、'
                  f'空セッション保存を避けるためスキップ', flush=True)
            return
        if not self.cli.service_is_ready():
            print(f'{time.strftime("%H:%M:%S")}  /slam_toolbox/serialize_map がまだ利用できません', flush=True)
            return
        self.count += 1
        fname = os.path.join(self.a.out_dir, f'checkpoint_{time.strftime("%H%M%S")}')
        req = SerializePoseGraph.Request()
        req.filename = fname
        future = self.cli.call_async(req)
        future.add_done_callback(lambda f: self._on_done(f, fname))

    def _on_done(self, future, fname):
        try:
            res = future.result()
            ok = (res.result == 0)
        except Exception as e:
            print(f'{time.strftime("%H:%M:%S")}  保存失敗(例外): {e}', flush=True)
            return
        status = 'OK' if ok else f'失敗(result={res.result})'
        print(f'{time.strftime("%H:%M:%S")}  checkpoint #{self.count} -> {fname} [{status}]', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--interval', type=float, default=60.0, help='保存間隔[秒]')
    ap.add_argument('--warmup', type=float, default=30.0,
                     help='起動後この秒数は保存しない(空セッションのSIGSEGV対策、Issue #902)')
    ap.add_argument('--out-dir', default=os.path.expanduser('~/ros2_ws/checkpoints'))
    a = ap.parse_args()
    rclpy.init()
    node = SlamCheckpoint(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
