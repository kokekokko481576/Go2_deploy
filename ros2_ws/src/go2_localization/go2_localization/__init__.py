import os

from ament_index_python.packages import get_package_share_directory


def default_map_yaml_path():
    """AMCLのmap_server既定値(cafe_world_map)の絶対パス。amcl.launch.py・
    localization.launch.py・demo.launch.pyがそれぞれ独立に組み立てて重複するのを
    避けるため、ここ1箇所にまとめる(Nav2/dev-up.sh側の`--map`地図選択とは別)。"""
    return os.path.join(
        get_package_share_directory('go2_localization'), 'config', 'map',
        'cafe_world_map.yaml')
