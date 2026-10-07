"""実機で地図を作る一式(static_tf / ekf / height_slice_viz / pointcloud_to_laserscan / slam_toolbox)。

**⚠ 実機では docker/driver/real_up.sh を使うこと。このlaunchのままでは床除去が効かず、
床を障害物として地図に焼く**(2026-09-04 実測で1m未満のビームが81.3%)。
height_slice_viz_real.launch.py が cloud_in=/utlidar/cloud・floor_z=-0.27(sim値) のままで、
実機では /utlidar/cloud_base_restamped・floor_z=-0.35 にする必要がある。
real_up.sh はこの2点を上書きして起動する(README「実機向け」参照)。
"""
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
