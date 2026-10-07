"""`arm_command`(rad) → D1 の JSON コマンド(度) の変換。**ROS に依存しない。**

実機もROSも無しでテストできるようにノードから分けてある
（`marker_approach` が制御則を `turn_drive_turn.py` に分けているのと同じ作法）。
テストは `test/test_angle_conversion.py`。

仕様の出どころは `docker/driver/d1_sdk/src/*.cpp`。詳細は `../README.md`。
"""

import json
import math

# `arm_command` の並び（simの d1_arm_controller と同じ）。
# 6軸はラジアン、グリッパー2軸はメートル。
N_INPUT = 8
N_SERVO = 7          # D1 の angle0..angle6

# 公称可動域[度]（`docs/計画/アーム動作.md` §4-1）。J1/J4/J6 が ±135、J2/J3/J5 が ±90。
JOINT_LIMIT_DEG = (135.0, 90.0, 90.0, 135.0, 90.0, 135.0)

FUNCODE_MULTI_JOINT = 2
FUNCODE_ENABLE = 5
FUNCODE_ZERO = 7

# グリッパー(angle6)の既定の固定値[度]。**実機ではグリッパーでカメラの支柱を挟んでおり、
# この値から変えるとカメラが落ちる。** カメラを付け直すと値が変わる:
#   2026-09-28 D435i: 13.2 / 2026-10-01 D405 に交換して挟み直し: -12.6（今の値）
# #78 の inspect_poses.json の gripper と必ず同じ値にすること。
DEFAULT_GRIPPER_FIXED_DEG = -12.6


def to_servo_degrees(values, signs, offsets,
                     gripper_open_m=0.033, gripper_closed_deg=0.0, gripper_open_deg=0.0,
                     gripper_fixed_deg=DEFAULT_GRIPPER_FIXED_DEG):
    """`arm_command`(rad + m) を D1 の angle0..angle6(度) へ変換する。

    戻り値は `(角度7個, 警告の文字列リスト)`。可動域を超えた分はクランプし、
    何をクランプしたかを警告として返す（ログに出すのは呼び出し側の仕事）。

    `gripper_fixed_deg` が None でなければ、グリッパーの入力を無視して angle6 をその値に
    固定する(既定)。funcode 2 は angle0..angle6 を必ず含むので「送らない」ことはできず、
    何かは送ることになる。対応が未確定のまま入力から計算すると、その値へ能動的に駆動してしまう。
    None のときだけ、開き量[m]を開閉の角度範囲へ線形に割り当てる(**この対応は推測**)。
    """
    out, warnings = [], []
    for i in range(6):
        deg = math.degrees(values[i]) * signs[i] + offsets[i]
        lim = JOINT_LIMIT_DEG[i]
        if abs(deg) > lim:
            warnings.append(
                f'J{i + 1} の指令 {deg:+.1f}度 が可動域 ±{lim:.0f}度 を超えています。クランプします')
            deg = math.copysign(lim, deg)
        out.append(deg)

    if gripper_fixed_deg is not None:
        out.append(gripper_fixed_deg)
        return out, warnings

    # グリッパー: simの2軸(左右)のうち開き量の大きい方を採り、0..1 に正規化する
    opening = max(values[6], values[7])
    ratio = 0.0 if gripper_open_m <= 0 else min(max(opening / gripper_open_m, 0.0), 1.0)
    out.append(gripper_closed_deg + ratio * (gripper_open_deg - gripper_closed_deg))
    return out, warnings


def build_payload(seq, address, funcode, data=None):
    """D1 が受け取る JSON 文字列を組み立てる。

    **separators を詰める。** SDK のサンプルが空白なしの JSON を送っており、
    機体側のパーサが空白を許すか確認できていないため、実物に寄せる。
    """
    payload = {'seq': seq, 'address': address, 'funcode': funcode}
    if data is not None:
        payload['data'] = data
    return json.dumps(payload, separators=(',', ':'))


def multi_joint_data(mode, angles):
    """funcode 2 の data 部。`{"mode":1,"angle0":..,...,"angle6":..}`"""
    return dict({'mode': mode}, **{f'angle{i}': round(a, 2) for i, a in enumerate(angles)})
