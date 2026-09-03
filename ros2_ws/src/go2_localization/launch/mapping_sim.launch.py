import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """simでmapping_real.launch.py(実機向け未検証コード)と同じ構成を検証するための、
    9/4の実機作業前のリハーサル用launch。既存sim版のEKF・height_slice_viz・
    pointcloud_to_laserscanをそのまま使い、AMCLの代わりにslam_toolboxで地図を作る。
    localization.launch.py(AMCL版)とは同時に起動しない(mapとodom->base_linkの
    配信元が競合する)。
    """
    pkg_share = get_package_share_directory('go2_localization')
    launch_dir = os.path.join(pkg_share, 'launch')

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py'))
    )
    height_slice_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'height_slice_viz.launch.py'))
    )
    p2l_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'pointcloud_to_laserscan.launch.py'))
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_sim.launch.py'))
    )

    return LaunchDescription([
        ekf_launch, height_slice_viz_launch, p2l_launch, slam_launch,
    ])
