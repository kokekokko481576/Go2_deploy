"""Go2_deploy 統合bringup launch(dev-up.sh から呼ばれる想定)。

自作サブシステム(自己位置推定 / 経路生成 / 経路追従)を、引数で on/off しながら
「まとめて1プロセスで起動」するための統合launch。個々の内部は知らなくても
ブラックボックスとして起動できることを狙う(scripts/dev-up.sh が対話で引数を決める)。

data flow(フェーズB配線・全ノード共通):
  /goal_pose
    → straight_line_planner (/plan を配信)          [use_planner]
    → plan_follower (/plan を FollowPath アクションへ) [常時]
    → controller_server (cmd_vel → /cmd_vel_raw)     [use_following]
    → cmd_vel_safety (/cmd_vel_raw → /robot1/cmd_vel) [常時]
    → ロボット

自己位置推定は「/go2_localization/tf を誰が供給するか」だけを決める:
  use_localization:=true  → 実装済み EKF/AMCL(go2_localization)が供給
  use_localization:=false → 起動しない。自作の推定が /go2_localization/tf を出す前提

引数(既定は全て true):
  use_localization : 実装済み自己位置推定(go2_localization)を起動するか
  use_planner      : 経路生成の見本(straight_line_planner)を起動するか
  use_following    : 経路追従(controller_server + lifecycle)を起動するか
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# フェーズB: 経路生成・controller とも自作TFを参照する。素の /tf には誰も配信しないため必須。
LOCALIZATION_TF = '/go2_localization/tf'
SIM_TIME = {'use_sim_time': True}


def generate_launch_description():
    use_localization = LaunchConfiguration('use_localization')
    use_planner = LaunchConfiguration('use_planner')
    use_following = LaunchConfiguration('use_following')

    loc_share = get_package_share_directory('go2_localization')
    follow_share = get_package_share_directory('go2_path_following')

    # --- 自己位置推定(実装済み EKF/AMCL)。/go2_localization/tf を供給 ---
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'localization.launch.py')),
        condition=IfCondition(use_localization),
    )

    # --- 経路生成の見本。goal_pose → 直線 Path(/plan)。自作TFへの remap が必須 ---
    planner = Node(
        package='straight_line_planner',
        executable='straight_line_planner_node',
        name='straight_line_planner_node',
        output='screen',
        parameters=[SIM_TIME],
        remappings=[('/tf', LOCALIZATION_TF)],
        condition=IfCondition(use_planner),
    )

    # --- 経路追従の本体(controller_server + lifecycle_manager) ---
    following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follow_share, 'launch', 'controller.launch.py')),
        condition=IfCondition(use_following),
    )

    # --- 橋渡し: /plan を FollowPath ゴールへ。controller_server とセットの付属品(常時起動) ---
    plan_follower = Node(
        package='go2_path_following',
        executable='plan_follower',
        name='plan_follower',
        output='screen',
        parameters=[SIM_TIME],
    )

    # --- 安全弁: /cmd_vel_raw をクランプ+ウォッチドッグして /robot1/cmd_vel へ中継(常時起動) ---
    cmd_vel_safety = Node(
        package='cmd_vel_safety',
        executable='cmd_vel_safety_node',
        name='cmd_vel_safety_node',
        output='screen',
        parameters=[SIM_TIME],
        remappings=[('cmd_vel', '/robot1/cmd_vel')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_localization', default_value='true'),
        DeclareLaunchArgument('use_planner', default_value='true'),
        DeclareLaunchArgument('use_following', default_value='true'),
        localization,
        planner,
        following,
        plan_follower,
        cmd_vel_safety,
    ])
