import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    # height_slice_vizの`ros__parameters`ブロックをこのyamlから直接読む
    p2l_config = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')

    viz_node = Node(
        package='go2_localization',
        executable='height_slice_viz',
        name='height_slice_viz',
        output='screen',
        parameters=[p2l_config],
        remappings=[
            ('cloud_in', '/robot1/chin_lidar/scan/points'),
            ('cloud_filtered', '/go2_localization/chin_lidar_scan_points'),
            ('/tf', '/robot1/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
    )

    return LaunchDescription([viz_node])
