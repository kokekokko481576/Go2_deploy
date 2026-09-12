"""`arm_command`(ROS2) を D1-T が理解する JSON コマンドへ変換して実機へ送るブリッジ。

Issue #64。`d1_arm_demo`(sim/実機共通の「角度を決めるロジック」)の**送信先だけを
差し替える**ための層（`docs/計画/アーム動作.md` §3-1）。

    d1_arm_demo ──arm_command(Float64MultiArray, rad)──▶ このノード
                                                          │ JSONへ変換
                                                          ▼
                                        /arm_Command (unitree_arm/ArmString)
                                                          │ ROS2がDDS名へ変換
                                                          ▼
                                                    rt/arm_Command → D1-T

## なぜ unitree_sdk2 を使わずROS2だけで書けるのか

D1 SDK のサンプル(`docker/driver/d1_sdk/src/*.cpp`)は CycloneDDS の生トピック
`rt/arm_Command` に `unitree_arm::msg::dds_::ArmString_`(文字列1個)を publish している。

**ROS2 のトピック `/arm_Command` は、DDS 上では `rt/arm_Command` になる**
(`rt/` は ROS2 が付ける接頭辞。Go2 本体の `/api/sport/request` ↔ `rt/api/sport/request`
と同じ仕組み)。そして ROS2 のパッケージ `unitree_arm` のメッセージ `ArmString` は
DDS 型名 `unitree_arm::msg::dds_::ArmString_`、フィールド `data_` を生成する。
**SDK 側の型と完全に一致する**ので、ROS2 のパブリッシャがそのまま機体のサブスクライバと
マッチする。

この方式を選んだ理由は ABI 衝突の回避。1つのプロセスで rclcpp(ROS2 の CycloneDDS)と
unitree_sdk2(`/usr/local/lib` の純正 CycloneDDS)を両方リンクすると、
2026-08-28 に実機で踏んだ `free(): invalid pointer` がそのまま再発する
(`docker/driver/d1_sdk/run.sh` が `LD_LIBRARY_PATH` を細工しているのはこのため)。
**ROS2 側の CycloneDDS だけを使えばこの問題は起きない。**

**DDS の層までは実機なしで検証済み**（2026-09-13）。unitree_sdk2 で生の
`rt/arm_Command` を購読する検査プログラム（`tools/dds_probe.cpp`）を機体と同じ立場に
立てたところ、このノードが送った JSON をそのまま受信した。トピック名・型名・
フィールド名の一致は確かめられている（手順は README）。

**残るのは実機固有の部分だけ**: 機体がこの JSON を受理するか、関節の回転方向、
グリッパー `angle6` の対応。外れた場合の代替は README の「うまくいかなかったら」。

## コマンドの形式(SDK のサンプルから読み取った実物)

| funcode | 用途 | data |
|---|---|---|
| 1 | 単一関節 | `{"id":5,"angle":60,"delay_ms":0}` |
| 2 | 複数関節 | `{"mode":1,"angle0":0,...,"angle6":0}` |
| 5 | 有効化/無効化 | `{"mode":0}` |
| 7 | ゼロ姿勢へ | (なし) |

**角度の単位は度**（`restore_initial_pose.cpp` が `91.4` `-89.3` を送っている）。
`arm_command` はラジアン(simと同じ)で受けるので、ここで度へ変換する。

## 安全のための既定値

- **`dry_run` は既定で true**。JSONをログに出すだけで送らない。実機へ出すときだけ
  明示的に false にする
- 可動域を超える角度はクランプする(J1/J4/J6 ±135度、J2/J3/J5 ±90度。公称スペック)
- `min_command_interval` で送信間隔の下限を設ける。D1 は連続コマンドで数分後に
  無応答化するという報告があり(`docs/計画/アーム動作.md` §4-3)、低頻度の離散コマンドを
  基本方針にしている
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

from unitree_arm.msg import ArmString

# `arm_command` の並び（simの d1_arm_controller と同じ）。
# 6軸はラジアン、グリッパー2軸はメートル。
N_INPUT = 8
N_SERVO = 7          # D1 の angle0..angle6

# 公称可動域[度]（`docs/計画/アーム動作.md` §4-1）。J1/J4/J6 が ±135、J2/J3/J5 が ±90。
JOINT_LIMIT_DEG = (135.0, 90.0, 90.0, 135.0, 90.0, 135.0)

FUNCODE_MULTI_JOINT = 2
FUNCODE_ENABLE = 5
FUNCODE_ZERO = 7


class ArmBridgeNode(Node):

    def __init__(self):
        super().__init__('d1_arm_bridge_node')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('mode', 1)
        self.declare_parameter('address', 1)
        self.declare_parameter('min_command_interval', 1.0)
        # 実機の関節回転方向は未照合。ベンダーURDFで2軸が実機と食い違っていたという
        # 他ラボの報告があるので（`docs/計画/アーム動作.md` §4-5）、符号とオフセットを
        # パラメータで直せるようにしておく。**実機で目視照合してから埋めること。**
        self.declare_parameter('joint_signs', [1.0] * 6)
        self.declare_parameter('joint_offsets_deg', [0.0] * 6)
        # グリッパーは sim が prismatic 2軸[m]、D1 は angle6 の1値。
        # 開き量[m] を 0..1 に正規化して、開閉の角度範囲へ線形に割り当てる。
        # **この対応は推測。** 実機で開閉させて実測してから直すこと。
        self.declare_parameter('gripper_open_m', 0.033)
        self.declare_parameter('gripper_closed_deg', 0.0)
        self.declare_parameter('gripper_open_deg', 0.0)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.mode = int(self.get_parameter('mode').value)
        self.address = int(self.get_parameter('address').value)
        self.min_interval = float(self.get_parameter('min_command_interval').value)
        self.signs = [float(x) for x in self.get_parameter('joint_signs').value]
        self.offsets = [float(x) for x in self.get_parameter('joint_offsets_deg').value]
        self.gripper_open_m = float(self.get_parameter('gripper_open_m').value)
        self.gripper_closed_deg = float(self.get_parameter('gripper_closed_deg').value)
        self.gripper_open_deg = float(self.get_parameter('gripper_open_deg').value)

        if len(self.signs) != 6 or len(self.offsets) != 6:
            raise SystemExit('joint_signs と joint_offsets_deg は6要素で渡してください')

        # **seq は毎回変える。** Go2 本体では `header.identity.id` を固定したまま
        # 同じ内容を送り続けると機体が重複とみなして無視する、という実機実測がある
        # (marker_detection の知見。前進効率 40%→83%)。D1 で同じ挙動をするかは未確認だが、
        # 固定にする理由が無いので増やしておく。
        self.seq = 0
        self.last_sent = None

        self.cmd_pub = self.create_publisher(ArmString, 'arm_command_out', 10)
        self.create_subscription(Float64MultiArray, 'arm_command', self.on_arm_command, 10)
        self.create_subscription(Bool, 'zero_pose', self.on_zero_pose, 10)

        self.get_logger().info(
            f'D1アームブリッジ 準備完了 mode={self.mode} address={self.address} '
            f'送信間隔の下限{self.min_interval}s '
            + ('**dry_run: JSONをログに出すだけで実機へ送りません**'
               if self.dry_run else '*** 実機へ送ります ***'))
        if not self.dry_run:
            self.get_logger().warn(
                'dry_run=false です。アームが実際に動きます。'
                '周囲の安全と可動範囲を確認してから指令を出してください')

    # ------------------------------------------------------------------ 受信

    def on_arm_command(self, msg):
        if len(msg.data) != N_INPUT:
            self.get_logger().error(
                f'arm_command の要素数が {len(msg.data)} です（{N_INPUT} を期待）。捨てます')
            return
        if not self._interval_ok():
            return

        angles = self._to_servo_degrees(list(msg.data))
        self._send(FUNCODE_MULTI_JOINT, dict(
            {'mode': self.mode},
            **{f'angle{i}': round(a, 2) for i, a in enumerate(angles)}))

    def on_zero_pose(self, msg):
        """ゼロ姿勢へ戻す（funcode 7）。安全な初期化・復帰用。"""
        if not msg.data:
            return
        if not self._interval_ok():
            return
        self._send(FUNCODE_ZERO, None)

    # ------------------------------------------------------------------ 変換

    def _to_servo_degrees(self, values):
        """`arm_command`(rad + m) を D1 の angle0..angle6(度) へ変換する。"""
        out = []
        for i in range(6):
            deg = math.degrees(values[i]) * self.signs[i] + self.offsets[i]
            lim = JOINT_LIMIT_DEG[i]
            if abs(deg) > lim:
                self.get_logger().warn(
                    f'J{i + 1} の指令 {deg:+.1f}度 が可動域 ±{lim:.0f}度 を超えています。'
                    'クランプします')
                deg = math.copysign(lim, deg)
            out.append(deg)

        # グリッパー: simの2軸(左右)のうち開き量の大きい方を採り、0..1 に正規化する
        opening = max(values[6], values[7])
        ratio = 0.0 if self.gripper_open_m <= 0 else min(max(opening / self.gripper_open_m, 0.0), 1.0)
        out.append(self.gripper_closed_deg
                   + ratio * (self.gripper_open_deg - self.gripper_closed_deg))
        return out

    # ------------------------------------------------------------------ 送信

    def _interval_ok(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_sent is not None and now - self.last_sent < self.min_interval:
            self.get_logger().warn(
                f'前回の送信から {now - self.last_sent:.2f}s しか経っていません'
                f'（下限 {self.min_interval}s）。この指令は捨てます')
            return False
        self.last_sent = now
        return True

    def _send(self, funcode, data):
        self.seq += 1
        payload = {'seq': self.seq, 'address': self.address, 'funcode': funcode}
        if data is not None:
            payload['data'] = data
        # **separators を詰める。** SDK のサンプルが空白なしの JSON を送っており、
        # 機体側のパーサが空白を許すか確認できていないため、実物に寄せる
        text = json.dumps(payload, separators=(',', ':'))

        if self.dry_run:
            self.get_logger().info(f'[dry_run] {text}')
            return
        self.cmd_pub.publish(ArmString(data=text))
        self.get_logger().info(f'送信: {text}')


def main():
    rclpy.init()
    node = ArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
