"""Nav2 footprint矩形が4箇所(go2_path_following/go2_path_planningのfootprint、
go2_localizationのbody_exclude_*、height_slice_viz.pyのdeclare_parameterデフォルト)に
手打ちされている問題(PR #69レビュー、Issue #70)に対する、ズレ検知用の軽量テスト。
共通化そのものは#70に先送りしたが、これがあれば少なくとも「更新漏れ」は壊れて分かる。
"""
import ast
import re

import yaml
from ament_index_python.packages import get_package_share_directory

_EXPECTED_MIN_X, _EXPECTED_MAX_X = -0.38, 0.37
_EXPECTED_MIN_Y, _EXPECTED_MAX_Y = -0.18, 0.18


def _load_yaml(package, relative_path):
    path = f'{get_package_share_directory(package)}/{relative_path}'
    with open(path) as f:
        return yaml.safe_load(f)


def _bounds_from_corners(corners_str):
    corners = ast.literal_eval(corners_str.strip())
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    return min(xs), max(xs), min(ys), max(ys)


def test_controller_server_footprint_matches_expected():
    data = _load_yaml('go2_path_following', 'config/controller_server.yaml')
    footprint_str = data['/**']['local_costmap']['local_costmap']['ros__parameters']['footprint']
    assert _bounds_from_corners(footprint_str) == (
        _EXPECTED_MIN_X, _EXPECTED_MAX_X, _EXPECTED_MIN_Y, _EXPECTED_MAX_Y)


def test_planner_server_footprint_matches_expected():
    data = _load_yaml('go2_path_planning', 'config/planner_server.yaml')
    footprint_str = data['/**']['global_costmap']['global_costmap']['ros__parameters']['footprint']
    assert _bounds_from_corners(footprint_str) == (
        _EXPECTED_MIN_X, _EXPECTED_MAX_X, _EXPECTED_MIN_Y, _EXPECTED_MAX_Y)


def test_pointcloud_to_laserscan_body_exclude_matches_expected():
    data = _load_yaml('go2_localization', 'config/pointcloud_to_laserscan.yaml')
    params = data['/**']['height_slice_viz']['ros__parameters']
    bounds = (params['body_exclude_min_x'], params['body_exclude_max_x'],
              params['body_exclude_min_y'], params['body_exclude_max_y'])
    assert bounds == (_EXPECTED_MIN_X, _EXPECTED_MAX_X, _EXPECTED_MIN_Y, _EXPECTED_MAX_Y)


def test_height_slice_viz_default_params_match_expected():
    # ソースの.pyを直接regexで読む(rclpy.init()せずに済ませるため。
    # コンテナ内で本番動作するのはyaml値の方で、これはyaml未指定の単体実行時の保険)
    import go2_localization.height_slice_viz as module
    with open(module.__file__) as f:
        src = f.read()

    def _default_of(param_name):
        m = re.search(
            r"declare_parameter\('%s',\s*(-?[0-9.]+)\)" % re.escape(param_name), src)
        assert m, f'{param_name} のdeclare_parameterが見つからない'
        return float(m.group(1))

    bounds = (
        _default_of('body_exclude_min_x'), _default_of('body_exclude_max_x'),
        _default_of('body_exclude_min_y'), _default_of('body_exclude_max_y'),
    )
    assert bounds == (_EXPECTED_MIN_X, _EXPECTED_MAX_X, _EXPECTED_MIN_Y, _EXPECTED_MAX_Y)
