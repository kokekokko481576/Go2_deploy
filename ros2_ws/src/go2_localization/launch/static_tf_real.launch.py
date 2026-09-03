from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link -> utlidar_lidar(実機ファームウェアの顎LiDAR frame_id)の静的TF。
    # 実測値が無いため、external/go2_ros2_sim_py/go2_description/xacro/robot.xacro の
    # chin_lidar_joint(sim用の仮値、実機到着後にキャリブレーション予定)の値を
    # 手打ちでそのまま流用している(xacro側から自動で読み込む仕組みは無い)。
    # 【重複注意】robot.xacro側のchin_lidar_jointのoriginを実測値で更新したら、
    # ここのx/y/z/roll/pitch/yawも手動で合わせて更新すること(ビルド時リンク無し)
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
