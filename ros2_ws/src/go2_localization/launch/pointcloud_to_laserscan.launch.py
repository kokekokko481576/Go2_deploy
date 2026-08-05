import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    p2l_config = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')

    p2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_config],
        remappings=[
            # Issue #56: 生の3D点群ではなく、height_slice_vizが理論床到達距離との比較で
            # 床を除去した後の点群を使う(単純な高さレンジだけでは床と壁を区別できない、
            # Issue #26)。height_slice_vizはlocalization.launch.pyで同時に起動される
            ('cloud_in', '/go2_localization/chin_lidar_scan_points'),
            ('scan', '/go2_localization/chin_lidar_scan'),
            ('/tf', '/robot1/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
    )

    return LaunchDescription([p2l_node])
