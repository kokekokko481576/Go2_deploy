import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_min_max_height():
    # pointcloud_to_laserscan.yamlのmin_height/max_heightをそのまま読み、
    # 可視化ノードとの手動同期(ズレ)を無くす
    pkg_share = get_package_share_directory('go2_localization')
    p2l_config = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')
    with open(p2l_config) as f:
        data = yaml.safe_load(f)
    params = data['/**']['pointcloud_to_laserscan']['ros__parameters']
    return params['min_height'], params['max_height']


def generate_launch_description():
    min_height, max_height = _load_min_max_height()

    viz_node = Node(
        package='go2_localization',
        executable='height_slice_viz',
        name='height_slice_viz',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'min_height': min_height,
            'max_height': max_height,
        }],
        remappings=[
            ('cloud_in', '/robot1/chin_lidar/scan/points'),
            ('cloud_filtered', '/go2_localization/chin_lidar_scan_points'),
            ('/tf', '/robot1/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
    )

    return LaunchDescription([viz_node])
