"""非常停止の停止指令を送る。estop.sh から呼ぶ。

`ros2 topic pub --once` を繰り返す形はやめた。publisher が DDS のマッチング前に
破棄されるとメッセージはどこにも届かず、実LANのマッチングは数百ms〜秒かかる。
1つのプロセスで publisher を生かしたまま、機体側の購読が見えてから一定時間送り続ける。
"""
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

from go2_sport_bridge.cmd_vel_to_sport_node import (
    ROBOT_SPORT_API_ID_MOVE,
    ROBOT_SPORT_API_ID_STOP_MOVE,
)

RATE_HZ = 20.0
# 購読が見えてから送り続ける時間
SEND_SEC = 3.0
# 購読が見えなくてもここで打ち切る(非常停止が黙ってハングするのは最悪)
GIVE_UP_SEC = 6.0
# StopMove を連投すると移動指令の受け付けやリモコン操作と競合する
# (cmd_vel_to_sport_node のウォッチドッグ参照)ので、1秒に1回に抑える
STOP_MOVE_PERIOD_SEC = 1.0


def main(args=None):
    # 送り終えるまでCtrl-Cで中断させない(数秒で必ず終わる)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Node('estop_send')
    pub = node.create_publisher(Request, '/api/sport/request', 10)

    def send(api_id, parameter=''):
        req = Request()
        # 同じidの指令は機体に重複として無視されるので毎回変える
        req.header.identity.id = time.time_ns()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        pub.publish(req)

    zero = json.dumps({'x': 0.0, 'y': 0.0, 'z': 0.0})
    start = time.monotonic()
    matched_at = None
    last_stop_move = None
    sent = 0
    while True:
        now = time.monotonic()
        if matched_at is None and pub.get_subscription_count() > 0:
            matched_at = now
        if matched_at is not None and now - matched_at >= SEND_SEC:
            break
        if now - start >= GIVE_UP_SEC:
            break
        send(ROBOT_SPORT_API_ID_MOVE, zero)
        if last_stop_move is None or now - last_stop_move >= STOP_MOVE_PERIOD_SEC:
            send(ROBOT_SPORT_API_ID_STOP_MOVE)
            last_stop_move = now
        sent += 1
        time.sleep(1.0 / RATE_HZ)

    if matched_at is None:
        print(f'[estop] {GIVE_UP_SEC:.0f}秒待っても /api/sport/request の購読者(機体)が見えなかった。'
              '停止指令は届いていない可能性が高い', file=sys.stderr)
    else:
        print(f'[estop] 購読者 {pub.get_subscription_count()} に停止指令を送った'
              f'(ゼロ速度Move {sent}回、マッチングまで {matched_at - start:.2f}s)')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if matched_at is not None else 1


if __name__ == '__main__':
    sys.exit(main())
