import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    # config自体はslam_real.launch.pyと共用(scan_topic等はsim/実機で同じ命名規則、
    # /go2_localization/chin_lidar_scan)。use_sim_time・/tfの向き先だけがsim固有
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_real.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': True}],
        remappings=[
            # AMCL(amcl.launch.py)と同じ理由: upstream Nav2の/robot1/tfと衝突しないよう
            # 専用トピックに分離する
            ('/tf', '/go2_localization/tf'),
            ('/tf_static', '/robot1/tf_static'),
            # AMCLのmap出力(/go2_localization/map)と同じ名前空間に揃える
            ('map', '/go2_localization/map'),
        ],
    )

    return LaunchDescription([slam_node])
