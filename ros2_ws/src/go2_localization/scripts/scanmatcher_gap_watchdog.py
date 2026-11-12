#!/usr/bin/env python3
"""scanmatcherの入力点群トピックの受信間隔を監視し、既知のping不安定(本体メイン基板.161、
断続的に接続が切れる)でデータが一定時間途切れたら、scanmatcher_nodeを自動で再起動する。

scanmatcherには途切れ後の姿勢復帰サービスが無く(2026-11-12確認)、途切れた状態で推定を
続けると姿勢が大きく破綻する(実機で観測済み: ロール-122.8度等)。再起動すれば地図は
失うが、姿勢が壊れたまま使い続けるよりは安全、という判断。

使い方: real_up.shのstart_scanmatcherから、scanmatcher本体と同じ環境・引数で起動される。
"""
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

GAP_THRESHOLD_SEC = 3.0
RESTART_COOLDOWN_SEC = 10.0
CHECK_PERIOD_SEC = 1.0
# ギャップ検知のたびにこのファイルへ書く(mtime更新)。ホスト側はこのファイルを
# bind mount経由で監視し、更新を見てtcpdumpを起動する(2026-11-12、ping不安定の
# 原因調査用。コンテナ内にはtcpdumpも無く、root権限も無いため、コンテナの中からは
# 直接パケットキャプチャできない。ホスト側に処理を委ねるためのファイル越しの合図)
GAP_TRIGGER_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.gap_trigger')
# real_up.shのPROCS表のscanmatcherパターンと同じ文字列。定数としてソース内に埋め込み、
# CLI引数にはしない。引数にすると、この文字列自体がwatchdog自身のps argsに載ってしまい、
# real_up.sh側のis_running("lib/scanmatcher/scanmatcher_node")がwatchdogをscanmatcher本体と
# 誤認してしまう(2026-11-12実装時に気付いて修正)
SCANMATCHER_PATTERN = 'lib/scanmatcher/scanmatcher_node'


class GapWatchdog(Node):
    def __init__(self, input_topic: str, restart_cmd: str):
        super().__init__('scanmatcher_gap_watchdog')
        self.restart_cmd = restart_cmd
        self.last_msg_time = time.monotonic()
        self.last_restart_time = 0.0
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, 10)
        self.timer = self.create_timer(CHECK_PERIOD_SEC, self._check_gap)
        self.get_logger().info(
            f"gap watchdog started: topic={input_topic} "
            f"gap_threshold={GAP_THRESHOLD_SEC}s restart_cooldown={RESTART_COOLDOWN_SEC}s")

    def _on_cloud(self, _msg):
        self.last_msg_time = time.monotonic()

    def _check_gap(self):
        now = time.monotonic()
        gap = now - self.last_msg_time
        if gap <= GAP_THRESHOLD_SEC:
            return
        if now - self.last_restart_time < RESTART_COOLDOWN_SEC:
            # 再起動直後の立ち上がり待ち中。二重に再起動しない
            return
        self.get_logger().warn(
            f"入力点群が{gap:.1f}秒途切れた(ping不安定の疑い)。scanmatcher_nodeを再起動する")
        self._write_gap_trigger(gap)
        self._restart_scanmatcher()
        self.last_restart_time = now
        # 再起動後、新プロセスからの最初のメッセージが来るまでの間に
        # 再度ゲートが引っかからないよう、タイマーを一旦リセットしておく
        self.last_msg_time = now

    def _write_gap_trigger(self, gap: float):
        try:
            with open(GAP_TRIGGER_FILE, 'w') as f:
                f.write(f"{time.time():.3f} {gap:.1f}\n")
        except OSError as e:
            self.get_logger().error(f"gap trigger書き込み失敗: {e}")

    def _restart_scanmatcher(self):
        subprocess.run(
            ['bash', '-c', f"pkill -9 -f '{SCANMATCHER_PATTERN}' 2>/dev/null; true"],
            check=False)
        time.sleep(1.0)
        subprocess.Popen(['bash', '-c', self.restart_cmd])


def main():
    if len(sys.argv) < 3:
        print("usage: scanmatcher_gap_watchdog.py <input_topic> <restart_cmd>",
              file=sys.stderr)
        sys.exit(1)
    input_topic, restart_cmd = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = GapWatchdog(input_topic, restart_cmd)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
