import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    launch_dir = os.path.join(pkg_share, 'launch')

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'static_tf_real.launch.py'))
    )
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf_real.launch.py'))
    )
    height_slice_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'height_slice_viz_real.launch.py'))
    )
    p2l_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'pointcloud_to_laserscan_real.launch.py'))
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_real.launch.py'))
    )

    return LaunchDescription([
        static_tf_launch,
        ekf_launch,
        height_slice_viz_launch,
        p2l_launch,
        slam_launch,
    ])
