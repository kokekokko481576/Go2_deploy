import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    amcl_config = os.path.join(pkg_share, 'config', 'amcl.yaml')
    map_yaml = os.path.join(pkg_share, 'config', 'map', 'cafe_world_map.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[amcl_config, {'yaml_filename': map_yaml}],
        remappings=[
            ('map', '/go2_localization/map'),
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
        remappings=[
            ('map', '/go2_localization/map'),
            ('amcl_pose', '/go2_localization/amcl_pose'),
            ('particlecloud', '/go2_localization/particlecloud'),
            # 初期姿勢の再指定入力(Issue #27、scripts/set_initial_pose.sh が使う)。
            # 他トピックと同様に専用名前空間へ分離しておく
            ('initialpose', '/go2_localization/initialpose'),
            # odom->base_link入力・map->odom出力とも自前のEKFと同じ専用トピックを使う
            # (upstreamの/robot1/tfとは分離)。静止TFはupstreamから読み続ける
            ('/tf', '/go2_localization/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[amcl_config],
    )

    return LaunchDescription([map_server_node, amcl_node, lifecycle_manager_node])
