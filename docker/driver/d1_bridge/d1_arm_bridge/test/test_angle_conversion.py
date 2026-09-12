"""`arm_command`(rad) → D1 の JSON コマンド(度) の変換を、実機もROSも無しで検証する。

ここで守りたいのは「実機に間違った角度を投げない」こと。特に可動域のクランプは、
外れると物理的にアームを壊しうるので機械的に押さえておく。

    pytest docker/driver/d1_bridge/d1_arm_bridge/test/
"""

import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from d1_arm_bridge.conversion import (  # noqa: E402
    FUNCODE_MULTI_JOINT, FUNCODE_ZERO, JOINT_LIMIT_DEG,
    build_payload, multi_joint_data, to_servo_degrees,
)

SIGNS = [1.0] * 6
OFFSETS = [0.0] * 6


def convert(values):
    return to_servo_degrees(values, SIGNS, OFFSETS)


def test_ラジアンが度に変換される():
    angles, warnings = convert([math.pi / 2, 0, 0, 0, 0, 0, 0, 0])
    assert angles[0] == pytest.approx(90.0)
    assert warnings == []


def test_負の角度がそのまま通る():
    """Gazeboで符号反転を疑った経緯があるので、明示的に押さえておく。"""
    angles, _ = convert([-0.5, -1.0, 0, 0, 0, 0, 0, 0])
    assert angles[0] == pytest.approx(math.degrees(-0.5))
    assert angles[1] == pytest.approx(math.degrees(-1.0))


def test_simの撮影姿勢が期待どおりの度になる():
    """`d1_arm_demo` の既定ウェイポイント（j1=1.57, j2=1.2）。

    この2つは Gazebo で先端が床から0.34m・俯角68.8度になることを実測した値なので、
    度への変換がずれるとsimと実機で違う姿勢になる。
    """
    angles, warnings = convert([1.57, 1.2, 0, 0, 0, 0, 0, 0])
    assert angles[0] == pytest.approx(89.95, abs=0.01)
    assert angles[1] == pytest.approx(68.75, abs=0.01)
    assert warnings == []


@pytest.mark.parametrize('index,limit', list(enumerate(JOINT_LIMIT_DEG)))
def test_可動域を超えた指令はクランプされる(index, limit):
    """J1/J4/J6 は ±135度、J2/J3/J5 は ±90度（公称スペック）。

    **これが外れると実機を壊しうる。** 全6軸を正負の両方で確認する。
    """
    for sign in (1.0, -1.0):
        values = [0.0] * 8
        values[index] = sign * math.radians(limit + 30.0)
        angles, warnings = convert(values)
        assert angles[index] == pytest.approx(sign * limit)
        assert len(warnings) == 1
        assert f'J{index + 1}' in warnings[0]


def test_可動域ちょうどはクランプされない():
    values = [0.0] * 8
    values[1] = math.radians(JOINT_LIMIT_DEG[1])
    angles, warnings = convert(values)
    assert angles[1] == pytest.approx(JOINT_LIMIT_DEG[1])
    assert warnings == []


def test_符号とオフセットが効く():
    """実機の回転方向が未照合なので、あとから符号で直せる必要がある。"""
    angles, _ = to_servo_degrees([math.radians(30), 0, 0, 0, 0, 0, 0, 0],
                                 [-1.0, 1, 1, 1, 1, 1], [5.0, 0, 0, 0, 0, 0])
    assert angles[0] == pytest.approx(-30.0 + 5.0)


def test_グリッパーは開いている方の軸を採って正規化される():
    """simは prismatic 2軸[m]、D1 は angle6 の1値。"""
    angles, _ = to_servo_degrees([0] * 6 + [0.0, 0.033], SIGNS, OFFSETS,
                                 gripper_open_m=0.033,
                                 gripper_closed_deg=0.0, gripper_open_deg=60.0)
    assert angles[6] == pytest.approx(60.0)

    angles, _ = to_servo_degrees([0] * 6 + [0.0165, 0.0], SIGNS, OFFSETS,
                                 gripper_open_m=0.033,
                                 gripper_closed_deg=0.0, gripper_open_deg=60.0)
    assert angles[6] == pytest.approx(30.0)


def test_グリッパーは開き量を超えても飽和する():
    angles, _ = to_servo_degrees([0] * 6 + [0.5, 0.0], SIGNS, OFFSETS,
                                 gripper_open_m=0.033,
                                 gripper_closed_deg=0.0, gripper_open_deg=60.0)
    assert angles[6] == pytest.approx(60.0)


def test_出力は7要素():
    """D1 の angle0..angle6 は7個。simの8要素(6軸+グリッパー2軸)から詰める。"""
    angles, _ = convert([0.0] * 8)
    assert len(angles) == 7


def test_JSONがSDKサンプルと同じ形になる():
    """`docker/driver/d1_sdk/src/multiple_joint_angle_control.cpp` が送っている形。

    空白を入れない・キーの順序・`data` の入れ子が実物と一致しているかを見る。
    """
    angles, _ = convert([0.0, math.radians(-60), math.radians(60), 0, math.radians(30), 0, 0, 0])
    text = build_payload(4, 1, FUNCODE_MULTI_JOINT, multi_joint_data(1, angles))
    assert ' ' not in text
    assert text.startswith('{"seq":4,"address":1,"funcode":2,"data":{"mode":1,')

    parsed = json.loads(text)
    assert parsed['data']['angle1'] == pytest.approx(-60.0)
    assert parsed['data']['angle2'] == pytest.approx(60.0)
    assert parsed['data']['angle4'] == pytest.approx(30.0)
    assert len([k for k in parsed['data'] if k.startswith('angle')]) == 7


def test_ゼロ姿勢のJSONにはdataが付かない():
    """`arm_zero_control.cpp` は `{"seq":4,"address":1,"funcode":7}` だけを送る。"""
    text = build_payload(9, 1, FUNCODE_ZERO)
    assert json.loads(text) == {'seq': 9, 'address': 1, 'funcode': 7}
    assert 'data' not in text
