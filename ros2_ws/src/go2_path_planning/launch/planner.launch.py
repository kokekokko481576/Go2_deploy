import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_path_planning')
    planner_config = os.path.join(pkg_share, 'config', 'planner_server.yaml')

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_config],
        remappings=[
            # controller.launch.py(go2_path_following)と同じフェーズB配線:
            # 自作AMCL/EKFが配信する専用TFを参照する
            ('/tf', '/go2_localization/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[planner_config],
    )

    return LaunchDescription([planner_server_node, lifecycle_manager_node])
