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
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))

# フェーズB: 経路生成・controller とも自作TFを参照する。素の /tf には誰も配信しないため必須。
LOCALIZATION_TF = '/go2_localization/tf'
# TF参照や sim時刻同期が要るノードだけ True。それ以外は /clock 購読コスト(#44)を避けて False。
SIM_TIME = {'use_sim_time': True}
NO_SIM_TIME = {'use_sim_time': False}


def generate_launch_description():
    use_localization = LaunchConfiguration('use_localization')
    planner_kind = LaunchConfiguration('planner')  # straight | dijkstra | none
    use_following = LaunchConfiguration('use_following')
    use_rviz = LaunchConfiguration('use_rviz')

    loc_share = get_package_share_directory('go2_localization')
    follow_share = get_package_share_directory('go2_path_following')
    plan_share = get_package_share_directory('go2_path_planning')

    def _planner_is(kind):
        return IfCondition(PythonExpression(["'", planner_kind, "' == '", kind, "'"]))

    # --- 自己位置推定(実装済み EKF/AMCL)。/go2_localization/tf を供給 ---
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'localization.launch.py')),
        condition=IfCondition(use_localization),
    )

    # --- 経路生成A(見本): 直線 Path を /plan へ。自作TFへの remap が必須 ---
    planner_straight = Node(
        package='straight_line_planner',
        executable='straight_line_planner_node',
        name='straight_line_planner_node',
        output='screen',
        parameters=[SIM_TIME],
        remappings=[('/tf', LOCALIZATION_TF)],
        condition=_planner_is('straight'),
    )

    # --- 経路生成B(本命): Nav2 planner_server(NavFn=ダイクストラ)。地図/コストマップから
    #     曲線 Path を計算し plan_requester が /plan へ流す。localization(map/TF)が前提 ---
    planner_dijkstra = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plan_share, 'launch', 'planner.launch.py')),
        condition=_planner_is('dijkstra'),
    )
    plan_requester = Node(
        package='go2_path_planning',
        executable='plan_requester',
        name='plan_requester',
        output='screen',
        parameters=[NO_SIM_TIME],
        condition=_planner_is('dijkstra'),
    )

    # --- 経路追従の本体(controller_server + lifecycle_manager) ---
    following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follow_share, 'launch', 'controller.launch.py')),
        condition=IfCondition(use_following),
    )

    # --- 橋渡し: /plan を FollowPath ゴールへ。タイマ/TF不使用なので sim時刻不要(#44) ---
    plan_follower = Node(
        package='go2_path_following',
        executable='plan_follower',
        name='plan_follower',
        output='screen',
        parameters=[NO_SIM_TIME],
    )

    # --- 安全弁: /cmd_vel_raw をクランプ+ウォッチドッグして /robot1/cmd_vel へ中継 ---
    #     ウォッチドッグの途絶検知は壁時計の方が安全側。sim時刻不要(#44)。
    cmd_vel_safety = Node(
        package='cmd_vel_safety',
        executable='cmd_vel_safety_node',
        name='cmd_vel_safety_node',
        output='screen',
        parameters=[NO_SIM_TIME],
        remappings=[('cmd_vel', '/robot1/cmd_vel')],
    )

    # --- 可視化用 map->odom 中継: /go2_localization/tf の map->odom を /robot1/tf へ ---
    #     これで /robot1/tf が map->odom->base_link->脚 の1本になり、RViz1つで骨も
    #     経路(/plan)もAMCL結果も出せる(#44)。制御パイプラインには影響しない(viz専用)。
    tf_relay = ExecuteProcess(
        cmd=['python3', os.path.join(_THIS_DIR, 'tf_map_odom_relay.py')],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # --- RViz(専用設定: Fixed Frame=odom, TF(base_link)/scan/経路/地図/AMCL を既定表示) ---
    #     /robot1/tf を読む。sim時刻のTFを扱うため use_sim_time=True 必須(壁時計だと破棄)。
    #     RVizはSave時に設定を全展開して上書きするので、ソースを汚さないよう毎回 /tmp へ複製し
    #     そのコピーを読ませる(RVizのSaveは/tmpに行き、ソースの既定は不変。#44)。
    src_rviz = os.path.join(_THIS_DIR, '..', 'rviz', 'go2_demo.rviz')
    active_rviz = '/tmp/go2_demo_active.rviz'
    try:
        shutil.copyfile(src_rviz, active_rviz)
    except OSError:
        active_rviz = src_rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', active_rviz],
        parameters=[SIM_TIME],
        remappings=[('/tf', '/robot1/tf'), ('/tf_static', '/robot1/tf_static')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_localization', default_value='true'),
        DeclareLaunchArgument(
            'planner', default_value='straight',
            description='経路生成: straight(直線見本) / dijkstra(NavFn曲線) / none'),
        DeclareLaunchArgument('use_following', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        localization,
        planner_straight,
        planner_dijkstra,
        plan_requester,
        following,
        plan_follower,
        cmd_vel_safety,
        tf_relay,
        rviz,
    ])
