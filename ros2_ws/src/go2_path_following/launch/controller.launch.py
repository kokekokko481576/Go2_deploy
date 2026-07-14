import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_path_following')
    controller_config = os.path.join(pkg_share, 'config', 'controller_server.yaml')

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_config],
        remappings=[
            ('cmd_vel', '/cmd_vel_raw'),
            # フェーズB: 自作AMCL/EKFが配信する専用TFを参照する
            # (フェーズAでは動作確認のため'/robot1/tf'を参照していた)
            ('/tf', '/go2_localization/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[controller_config],
    )

    return LaunchDescription([controller_server_node, lifecycle_manager_node])
