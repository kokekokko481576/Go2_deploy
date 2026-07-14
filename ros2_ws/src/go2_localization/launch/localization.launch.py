import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    launch_dir = os.path.join(pkg_share, 'launch')

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py'))
    )
    p2l_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'pointcloud_to_laserscan.launch.py'))
    )
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'amcl.launch.py'))
    )
    height_slice_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'height_slice_viz.launch.py'))
    )

    return LaunchDescription([ekf_launch, p2l_launch, amcl_launch, height_slice_viz_launch])
