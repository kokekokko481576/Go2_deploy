from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link -> utlidar_lidar(実機ファームウェアの顎LiDAR frame_id)の静的TF。
    # 2026-11-04、チェックリスト§4-1の実測値(巻尺+傾斜計アプリ)に更新。
    # Head_lower基準点(x=0.293, z=-0.06)からΔx=0, Δy=0, Δz=+0.05m、
    # ピッチ実測18°(=0.3142rad、仮値20°に近く妥当と判断)、ロール0°。
    # external/go2_ros2_sim_py/go2_description/xacro/robot.xacro の
    # chin_lidar_joint(xacro側から自動で読み込む仕組みは無い)も合わせて更新すること。
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_utlidar_lidar',
        output='screen',
        arguments=[
            '--x', '0.293', '--y', '0.0', '--z', '-0.01',
            '--roll', '0', '--pitch', '0.3142', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'utlidar_lidar',
        ],
    )

    return LaunchDescription([static_tf_node])
