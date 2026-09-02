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
        # config/pointcloud_to_laserscan.yamlはsim用にuse_sim_time: trueだが、
        # 実機は壁時計を使うため上書きする
        parameters=[p2l_config, {'use_sim_time': False}],
        remappings=[
            ('cloud_in', '/go2_localization/chin_lidar_scan_points'),
            ('scan', '/go2_localization/chin_lidar_scan'),
            # 実機側にはsimの/robot1名前空間が無いため素の/tf・/tf_staticを使う
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    return LaunchDescription([p2l_node])
