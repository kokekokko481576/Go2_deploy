import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    launch_dir = os.path.join(pkg_share, 'launch')
    default_map_yaml = os.path.join(pkg_share, 'config', 'map', 'cafe_world_map.yaml')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value=default_map_yaml,
        description='AMCLのmap_serverに渡す地図yamlの絶対パス(既定: cafe_world_map)',
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py'))
    )
    p2l_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'pointcloud_to_laserscan.launch.py'))
    )
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'amcl.launch.py')),
        launch_arguments={'map_yaml': LaunchConfiguration('map_yaml')}.items(),
    )
    height_slice_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'height_slice_viz.launch.py'))
    )

    return LaunchDescription([
        map_yaml_arg, ekf_launch, p2l_launch, amcl_launch, height_slice_viz_launch,
    ])
