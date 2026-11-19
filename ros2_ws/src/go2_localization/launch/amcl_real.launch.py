import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_localization import default_map_yaml_path


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    amcl_config = os.path.join(pkg_share, 'config', 'amcl.yaml')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value=default_map_yaml_path(),
        description='map_server に渡す地図yamlの絶対パス(既定: cafe_world_map)',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        # config/amcl.yamlはsim用にuse_sim_time: trueだが、実機は壁時計を使うため
        # 上書きする(ekf_real.launch.pyと同じ手法)
        parameters=[amcl_config, {'yaml_filename': LaunchConfiguration('map_yaml'),
                                   'use_sim_time': False}],
        remappings=[
            ('map', '/go2_localization/map'),
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config, {'use_sim_time': False}],
        remappings=[
            ('map', '/go2_localization/map'),
            ('amcl_pose', '/go2_localization/amcl_pose'),
            ('particlecloud', '/go2_localization/particlecloud'),
            ('initialpose', '/go2_localization/initialpose'),
            # 実機側にはsimの/robot1名前空間が無いため、ekf_real.launch.pyと同様に
            # remapせず既定の/tf・/tf_staticを使う(sim版amcl.launch.pyの
            # /go2_localization/tf remapはここでは行わない)
        ],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[amcl_config, {'use_sim_time': False}],
    )

    return LaunchDescription([map_yaml_arg, map_server_node, amcl_node, lifecycle_manager_node])
