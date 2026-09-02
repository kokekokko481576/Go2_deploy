"""Issue #56のPR #57レビューで指摘された、height_slice_vizの`_obstacle_mask`
(理論床到達距離との比較による床除去)に対する単体テスト。

`_obstacle_mask`はself.floor_z/floor_margin/max_height/body_exclude_min_x/
body_exclude_max_x/body_exclude_min_y/body_exclude_max_y の7属性しか参照しないため、
rclpy.init()やNode継承を経由せず、この7属性だけを持つスタブを`self`として渡して
直接呼び出せる。
"""
import numpy as np

from go2_localization.height_slice_viz import HeightSliceViz


class _FakeParams:

    def __init__(self, floor_z=-0.27, floor_margin=0.3, max_height=0.3,
                 body_exclude_min_x=-0.38, body_exclude_max_x=0.35,
                 body_exclude_min_y=-0.18, body_exclude_max_y=0.18):
        self.floor_z = floor_z
        self.floor_margin = floor_margin
        self.max_height = max_height
        self.body_exclude_min_x = body_exclude_min_x
        self.body_exclude_max_x = body_exclude_max_x
        self.body_exclude_min_y = body_exclude_min_y
        self.body_exclude_max_y = body_exclude_max_y


def _mask(points, sensor_origin=(0.0, 0.0, 0.0), **params):
    node = _FakeParams(**params)
    points = np.asarray(points, dtype=np.float64)
    sensor_origin = np.asarray(sensor_origin, dtype=np.float64)
    return HeightSliceViz._obstacle_mask(node, points, sensor_origin)


def test_point_exactly_on_floor_is_removed():
    point = np.array([[2.0, 0.0, -0.27]])
    assert not _mask(point)[0]


def test_closer_reflection_on_same_ray_is_kept_as_obstacle():
    # 同じレイ方向(仰角)のまま、理論上の床到達距離より手前の近距離で反射した点。
    # 壁など障害物からの反射に相当し、除去されてはならない
    ray = np.array([2.0, 0.0, -0.27])
    point = (ray * 0.5).reshape(1, 3)
    assert _mask(point)[0]


def test_margin_absorbs_reflection_just_short_of_theoretical_floor():
    floor_margin = 0.3
    ray = np.array([2.0, 0.0, -0.27])
    floor_range = np.linalg.norm(ray)
    shrink = (floor_range - floor_margin * 0.5) / floor_range
    point = (ray * shrink).reshape(1, 3)
    assert not _mask(point, floor_margin=floor_margin)[0]


def test_upward_ray_is_never_treated_as_floor():
    # ray_z >= 0 (水平以上)は床に当たらないレイなので常に障害物側
    point = np.array([[2.0, 0.0, 0.1]])
    assert _mask(point)[0]


def test_point_above_max_height_is_excluded_even_if_obstacle():
    point = np.array([[0.5, 0.0, 0.5]])
    assert not _mask(point, max_height=0.3)[0]


def test_sensor_origin_offset_is_respected():
    # chin_lidar_frame相当のオフセット(base_link原点からz=-0.06)を想定
    sensor_origin = (0.29, 0.0, -0.06)
    ray = np.array([2.0, 0.0, -0.27 - (-0.06)])
    point = (np.array(sensor_origin) + ray).reshape(1, 3)
    assert not _mask(point, sensor_origin=sensor_origin)[0]


def test_point_inside_body_footprint_is_excluded_even_if_obstacle():
    # 脚が前に振り出されてrange_min(0.4m)を超え、Nav2 footprint矩形内で反射した想定。
    # 床には当たらない(z>=0相当)近距離の反射だが、本体自身なので障害物扱いしない
    point = np.array([[0.3, 0.05, 0.1]])
    assert not _mask(point)[0]


def test_point_outside_body_footprint_is_still_kept_as_obstacle():
    # footprint矩形のすぐ外(x=0.4 > body_exclude_max_x=0.35)は通常どおり障害物扱い
    point = np.array([[0.4, 0.05, 0.1]])
    assert _mask(point)[0]
