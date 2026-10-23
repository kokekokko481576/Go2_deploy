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
- `min_command_interval`(既定25秒)で送信間隔の下限を設ける。D1 は連続コマンドで数分後に
  無応答化するという報告があり(`docs/計画/アーム動作.md` §4-3)、低頻度の離散コマンドを
  基本方針にしている。**下限に満たない指令は捨てずに待ち行列に積み、順番どおりに送る**
  (`pacer.py`。捨てると上流のウェイポイントが黙って抜ける)
- `zero_pose` は下限を無視してすぐ送り、待ち行列は捨てる(復帰したい瞬間に待たせない)
- **グリッパー(angle6)は既定で固定値 -12.6度を送る**（2026-10-01 に D405 へ交換して挟み直した値）。 実機ではグリッパーでカメラを挟んでおり、
  変えるとカメラが落ちる。指令に追従させるのは `gripper_follow_command:=true` のときだけ
"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

from unitree_arm.msg import ArmString

from .conversion import (
    DEFAULT_GRIPPER_FIXED_DEG, FUNCODE_MULTI_JOINT, FUNCODE_ZERO, N_INPUT,
    build_payload, multi_joint_data, to_servo_degrees,
)
from .pacer import CommandPacer

# 機体が購読しているトピック。DDS 上では rt/arm_Command になる。名前に選択の余地は無い
ARM_COMMAND_TOPIC = '/arm_Command'

class ArmBridgeNode(Node):

    def __init__(self):
        super().__init__('d1_arm_bridge_node')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('mode', 1)
        self.declare_parameter('address', 1)
        self.declare_parameter('min_command_interval', 25.0)
        self.declare_parameter('max_pending', 10)
        # 実機の関節回転方向は未照合。ベンダーURDFで2軸が実機と食い違っていたという
        # 他ラボの報告があるので（`docs/計画/アーム動作.md` §4-5）、符号とオフセットを
        # パラメータで直せるようにしておく。**実機で目視照合してから埋めること。**
        # `-1` のように整数で書いても通るよう、型を固定しない(後で float にする)
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('joint_signs', [1.0] * 6, any_type)
        self.declare_parameter('joint_offsets_deg', [0.0] * 6, any_type)
        # グリッパーは既定で固定値を送る(conversion.DEFAULT_GRIPPER_FIXED_DEG 参照。
        # 実機ではカメラを挟んでいて、変えると落ちる)。
        # 指令に追従させる場合は sim の prismatic 2軸[m] の開き量を 0..1 に正規化して、
        # 開閉の角度範囲へ線形に割り当てる。**この対応は推測。** 実機で開閉させて実測してから使うこと。
        self.declare_parameter('gripper_follow_command', False)
        self.declare_parameter('gripper_fixed_deg', DEFAULT_GRIPPER_FIXED_DEG)
        self.declare_parameter('gripper_open_m', 0.033)
        self.declare_parameter('gripper_closed_deg', 0.0)
        self.declare_parameter('gripper_open_deg', 0.0)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.mode = int(self.get_parameter('mode').value)
        self.address = int(self.get_parameter('address').value)
        self.min_interval = float(self.get_parameter('min_command_interval').value)
        self.max_pending = int(self.get_parameter('max_pending').value)
        self.signs = [float(x) for x in self.get_parameter('joint_signs').value]
        self.offsets = [float(x) for x in self.get_parameter('joint_offsets_deg').value]
        self.gripper_open_m = float(self.get_parameter('gripper_open_m').value)
        self.gripper_closed_deg = float(self.get_parameter('gripper_closed_deg').value)
        self.gripper_open_deg = float(self.get_parameter('gripper_open_deg').value)
        self.gripper_follow = bool(self.get_parameter('gripper_follow_command').value)
        self.gripper_fixed_deg = float(self.get_parameter('gripper_fixed_deg').value)

        if len(self.signs) != 6 or len(self.offsets) != 6:
            raise SystemExit('joint_signs と joint_offsets_deg は6要素で渡してください')
        if self.gripper_follow and self.gripper_open_deg == self.gripper_closed_deg:
            # 開閉の角度が同じだと、入力によらず常に同じ角度へ能動的に駆動する
            raise SystemExit('gripper_follow_command:=true には gripper_open_deg と '
                             'gripper_closed_deg に異なる値(実測値)を渡してください')

        # **seq は毎回変える。** Go2 本体では `header.identity.id` を固定したまま
        # 同じ内容を送り続けると機体が重複とみなして無視する、という実機実測がある
        # (marker_detection の知見。前進効率 40%→83%)。D1 で同じ挙動をするかは未確認だが、
        # 固定にする理由が無いので増やしておく。
        self.seq = 0
        self.pacer = CommandPacer(self.min_interval, self.max_pending)

        # 既定のトピック名を機体の購読と同じにしておく。別名を既定にすると remap を
        # 忘れたときに「送信:」がログに出るのに機体には何も届かない
        self.cmd_pub = self.create_publisher(ArmString, ARM_COMMAND_TOPIC, 10)
        self.create_subscription(Float64MultiArray, 'arm_command', self.on_arm_command, 10)
        self.create_subscription(Bool, 'zero_pose', self.on_zero_pose, 10)
        self.create_timer(0.2, self._drain)

        self.get_logger().info(
            f'D1アームブリッジ 準備完了 mode={self.mode} address={self.address} '
            f'出力={self.cmd_pub.topic_name} 送信間隔の下限{self.min_interval}s '
            + (f'グリッパー=指令に追従({self.gripper_closed_deg}〜{self.gripper_open_deg}度) '
               if self.gripper_follow else f'グリッパー=固定{self.gripper_fixed_deg}度 ')
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

        angles, warnings = to_servo_degrees(
            list(msg.data), self.signs, self.offsets,
            self.gripper_open_m, self.gripper_closed_deg, self.gripper_open_deg,
            None if self.gripper_follow else self.gripper_fixed_deg)
        for w in warnings:
            self.get_logger().warn(w)
        if not self.pacer.push(angles):
            self.get_logger().error(
                f'送信待ちが {self.max_pending} 件たまっています。この指令は捨てます'
                '（上流の送信間隔が min_command_interval より短すぎます）')
            return
        wait = self.pacer.wait_time(self._now())
        if wait > 0.0:
            self.get_logger().info(
                f'送信間隔の下限まで {wait:.1f}s 待ってから送ります（待ち {len(self.pacer.pending)} 件）')
        self._drain()

    def on_zero_pose(self, msg):
        """ゼロ姿勢へ戻す（funcode 7）。安全な初期化・復帰用。

        送信間隔の下限を無視してすぐ送る。戻したい瞬間に最大で下限ぶん待たせないため。
        """
        if not msg.data:
            return
        dropped = self.pacer.bypass(self._now())
        if dropped:
            self.get_logger().warn(f'送信待ちの指令 {dropped} 件を捨てて、ゼロ姿勢へ戻します')
        self._send(FUNCODE_ZERO, None)

    # ------------------------------------------------------------------ 送信

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _drain(self):
        angles = self.pacer.pop_ready(self._now())
        if angles is not None:
            self._send(FUNCODE_MULTI_JOINT, multi_joint_data(self.mode, angles))

    def _send(self, funcode, data):
        self.seq += 1
        text = build_payload(self.seq, self.address, funcode, data)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGTERM では rclpy が先にコンテキストを畳むので、二重に shutdown しない
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
