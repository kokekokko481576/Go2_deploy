import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('raw_odom_input', '/robot1/odometry/filtered'),
            ('imu_plugin/out', '/robot1/imu_plugin/out'),
            ('odometry/filtered', '/go2_localization/odometry/filtered'),
            ('/tf', '/robot1/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
    )

    return LaunchDescription([ekf_node])
