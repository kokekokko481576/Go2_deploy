from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link -> utlidar_lidar(実機ファームウェアの顎LiDAR frame_id)の静的TF。
    # 実測値が無いため、go2_description/xacro/robot.xacro のchin_lidar_joint
    # (sim用の仮値、実機到着後にキャリブレーション予定)をそのまま流用する。
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_utlidar_lidar',
        output='screen',
        arguments=[
            '--x', '0.29', '--y', '0.0', '--z', '-0.06',
            '--roll', '0', '--pitch', '0.35', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'utlidar_lidar',
        ],
    )

    return LaunchDescription([static_tf_node])
