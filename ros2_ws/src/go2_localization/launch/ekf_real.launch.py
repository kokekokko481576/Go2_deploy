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
        # config/ekf.yamlはsim用にuse_sim_time: trueだが、実機は壁時計を使うため
        # ここで上書きする(後の要素ほど優先。amcl.launch.pyのyaml_filename上書きと同じ手法)
        parameters=[ekf_config, {'use_sim_time': False}],
        remappings=[
            # sim版(ekf.launch.py)の/robot1/odometry/filtered・/robot1/imu_plugin/outに相当。
            # 実機は自前でOdometry/Imuを配信していないため、driverコンテナ側のノードで用意する。
            #
            # 2026-11-03: raw_odom_inputは当初 state_to_odom_imu_node(sportmodestate由来、
            # 関節エンコーダ+IMU)の/go2_state_bridge/odomを使っていたが、これで作った地図が
            # IMU姿勢補正後もSLAM中に大きく歪んでいた(オドメトリドリフトを疑う)。
            # ファームウェアが別途配信する/utlidar/robot_odom(LiDARベースと見られる、
            # utlidar_odom_restamp_nodeで機体クロック→開発PC壁時計に打ち直し済み)に切り替えて
            # 精度を比較検証中。改善しなければ/go2_state_bridge/odomに戻すこと
            ('raw_odom_input', '/utlidar/robot_odom_restamped'),
            ('imu_plugin/out', '/go2_state_bridge/imu'),
            ('odometry/filtered', '/go2_localization/odometry/filtered'),
            # 実機側にはsimの/robot1名前空間・上位Nav2スタックが無いため、
            # namespace分離(sim版の/go2_localization/tf)は不要。remapせず既定の
            # /tf・/tf_staticへそのまま配信する
        ],
    )

    return LaunchDescription([ekf_node])
