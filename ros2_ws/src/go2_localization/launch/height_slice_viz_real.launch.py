import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    p2l_config = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')

    viz_node = Node(
        package='go2_localization',
        executable='height_slice_viz',
        name='height_slice_viz',
        output='screen',
        # config/pointcloud_to_laserscan.yamlはsim用にuse_sim_time: trueだが、
        # 実機は壁時計を使うため上書きする
        parameters=[p2l_config, {'use_sim_time': False}],
        remappings=[
            # sim版(height_slice_viz.launch.py)の/robot1/chin_lidar/scan/pointsに相当。
            # 実機ファームウェアが直接配信する点群(unitree_ros2 README「utlidar/cloud」)
            ('cloud_in', '/utlidar/cloud'),
            ('cloud_filtered', '/go2_localization/chin_lidar_scan_points'),
            # 実機側にはsimの/robot1名前空間が無いため、remapせず既定の/tf・/tf_staticを使う
        ],
    )

    return LaunchDescription([viz_node])
