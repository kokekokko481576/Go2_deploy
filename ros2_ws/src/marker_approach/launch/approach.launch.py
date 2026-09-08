"""接近制御(marker_approach) + 安全フィルタ(cmd_vel_safety) を起動する。

**マーカー姿勢の供給元はこのlaunchに含まない。** 環境ごとに違うため:

  - Gazebo : `apriltag_ros` + `docker/sim/tools/sim_tag_bridge.py`
             （`docker/sim/tools/sim_up.sh` が一式を起動する。**simではこのlaunchは使わない**——
               接近制御はJazzyのsimコンテナ内で動かす必要があるため。理由はREADME参照）
  - 実機   : マーカー検出ノード（未取り込み。#74 の実機側で追って入れる）

このlaunchは `marker_pose`(PoseStamped) と `marker_diagnostics`(DiagnosticArray) を
外から受ける前提で、接近制御と安全フィルタだけを立てる。

起動しただけでは動かない。有効化は別途:
  ros2 topic pub --once /marker_approach_node/enable std_msgs/msg/Bool "{data: true}"

出力は `cmd_vel_raw` -> (cmd_vel_safety) -> `cmd_vel`。
**必ず cmd_vel_safety を挟んでからドライバへ渡すこと**（速度・加速度クランプと
ウォッチドッグはあちらの責務。接近制御側に重複して持たせない）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('dry_run', default_value='true',
                              help='trueなら速度指令を出さず計算結果をログに出すだけ'),
        DeclareLaunchArgument('standoff', default_value='0.65',
                              help='マーカーからどれだけ手前に立つか[m]。'
                                   '150mmタグでは0.58mを切れない（視野からはみ出す）'),
        # ゴールをどこに置くか。true: マーカー法線上（面に正対して止まる）。
        # false: 視線上＝マーカーの手前。横へ寄らないので届かない配置が無くなる代わりに、
        # 法線からのずれが残ったまま斜めに止まる。
        DeclareLaunchArgument('use_normal', default_value='true'),
        # 到達後の最終姿勢。right/left でマーカーを真横に入れる旋回を足す（アーム作業用）。
        # **その旋回はマーカーが視野から出るのでヨー角が要る**（yaw_source）。
        DeclareLaunchArgument('final_heading', default_value='marker'),
        DeclareLaunchArgument('side_turn_angle_deg', default_value='90.0'),
        DeclareLaunchArgument('yaw_source', default_value='none'),
        DeclareLaunchArgument('yaw_topic', default_value='/odom'),
        DeclareLaunchArgument('marker_pose_topic', default_value='/marker_pose'),
        DeclareLaunchArgument('marker_diag_topic', default_value='/marker_diagnostics'),
    ]
    approach = Node(
        package='marker_approach', executable='approach_node',
        name='marker_approach_node', output='screen',
        parameters=[{'dry_run': LaunchConfiguration('dry_run'),
                     'standoff': LaunchConfiguration('standoff'),
                     'use_normal': LaunchConfiguration('use_normal'),
                     'final_heading': LaunchConfiguration('final_heading'),
                     'side_turn_angle_deg': LaunchConfiguration('side_turn_angle_deg'),
                     'yaw_source': LaunchConfiguration('yaw_source'),
                     'yaw_topic': LaunchConfiguration('yaw_topic')}],
        remappings=[('marker_pose', LaunchConfiguration('marker_pose_topic')),
                    ('marker_diagnostics', LaunchConfiguration('marker_diag_topic'))])
    # 横速度の上限は0。turn-drive-turn は横移動を使わない（混ざると前進が止まる）
    safety = Node(
        package='cmd_vel_safety', executable='cmd_vel_safety_node',
        name='cmd_vel_safety_node', output='screen',
        parameters=[{'max_linear_x': 0.22, 'max_linear_y': 0.0,
                     'max_angular_z': 0.45, 'max_linear_accel': 0.3,
                     'max_angular_accel': 0.6, 'watchdog_timeout': 0.3}])
    return LaunchDescription(args + [approach, safety])
