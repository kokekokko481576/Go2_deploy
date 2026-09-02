import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_real.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
        remappings=[
            # AMCLのmap出力(/go2_localization/map)と同じ名前空間に揃える
            ('map', '/go2_localization/map'),
        ],
    )

    return LaunchDescription([slam_node])
