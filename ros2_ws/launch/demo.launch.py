"""Go2_deploy 統合bringup launch(dev-up.sh から呼ばれる想定)。

自作サブシステム(自己位置推定 / 経路生成 / 経路追従)を、引数で on/off しながら
「まとめて1プロセスで起動」するための統合launch。個々の内部は知らなくても
ブラックボックスとして起動できることを狙う(scripts/dev-up.sh が対話で引数を決める)。

data flow(フェーズB配線・全ノード共通。BT導入(#23派生)後):
  /goal_pose
    → goal_pose_bridge (NavigateToPoseアクションへ)      [常時]
    → bt_navigator (go2_bt_plugins/behavior_trees/navigate_with_collision_recovery.xml)
        - ComputePathToPose → planner_server(go2_path_planning、ダイクストラ)
        - FollowPath → controller_server(go2_path_following、DWB)
        - collision_detected(#23、LiDAR+IMU/オドメトリ)を検知したらFollowPathを止め、
          Spin/Wait/BackUp(behavior_server)のリカバリへ落ちる
    → cmd_vel_safety (/cmd_vel_raw → /robot1/cmd_vel)     [常時]
    → ロボット

経路生成は常にダイクストラ(go2_path_planning、obstacle_layer込み)を使う。
直線見本(straight_line_planner)は、地図/障害物を無視するため後退後も同じ壁へ
直進を繰り返すことが判明した(#23)ので、既定のnavigate flowからは外した。
単体で試したい場合はstraight_line_planner_nodeを個別に起動すること。

自己位置推定は「/go2_localization/tf を誰が供給するか」だけを決める:
  use_localization:=true  → 実装済み EKF/AMCL(go2_localization)が供給
  use_localization:=false → 起動しない。自作の推定が /go2_localization/tf を出す前提

引数(既定は全て true):
  use_localization : 実装済み自己位置推定(go2_localization)を起動するか
  use_following    : 経路追従(controller_server + lifecycle)を起動するか
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))

# フェーズB: 経路生成・controller・BTとも自作TFを参照する。素の/tfには誰も配信しないため必須。
LOCALIZATION_TF = '/go2_localization/tf'
TF_REMAP = [('/tf', LOCALIZATION_TF), ('/tf_static', '/robot1/tf_static')]
# TF参照やsim時刻同期が要るノードだけTrue。それ以外は/clock購読コスト(#44)を避けてFalse。
SIM_TIME = {'use_sim_time': True}
NO_SIM_TIME = {'use_sim_time': False}


def generate_launch_description():
    use_localization = LaunchConfiguration('use_localization')
    use_following = LaunchConfiguration('use_following')
    use_rviz = LaunchConfiguration('use_rviz')

    loc_share = get_package_share_directory('go2_localization')
    follow_share = get_package_share_directory('go2_path_following')
    plan_share = get_package_share_directory('go2_path_planning')
    bt_plugins_share = get_package_share_directory('go2_bt_plugins')

    bt_xml_path = os.path.join(
        bt_plugins_share, 'behavior_trees', 'navigate_with_collision_recovery.xml')
    bt_navigator_config = os.path.join(follow_share, 'config', 'bt_navigator.yaml')
    behavior_server_config = os.path.join(follow_share, 'config', 'behavior_server.yaml')

    # --- 自己位置推定(実装済み EKF/AMCL)。/go2_localization/tf を供給 ---
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'localization.launch.py')),
        condition=IfCondition(use_localization),
    )

    # --- 経路生成(ダイクストラ): Nav2 planner_server。bt_navigatorのComputePathToPoseが
    #     直接このアクションを呼ぶ(旧plan_requesterは不要になった) ---
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plan_share, 'launch', 'planner.launch.py')),
    )

    # --- 経路追従の本体(controller_server + lifecycle_manager) ---
    following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follow_share, 'launch', 'controller.launch.py')),
        condition=IfCondition(use_following),
    )

    # --- BTナビゲーション(#23派生): 経路生成→追従→リカバリの判断を全てBTに持たせる ---
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        # NavigateThroughPosesは使わないが、bt_navigatorはconfigure時に両方のデフォルトBTを
        # 検証するため、stock版の代わりに同じ単一ゴール用ツリーを割り当てておく
        parameters=[bt_navigator_config, {
            'default_nav_to_pose_bt_xml': bt_xml_path,
            'default_nav_through_poses_bt_xml': bt_xml_path,
        }],
        remappings=TF_REMAP,
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behavior_server_config],
        remappings=TF_REMAP,
    )
    lifecycle_manager_bt = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_bt',
        output='screen',
        parameters=[behavior_server_config],
    )

    # --- 橋渡し: /goal_pose をNavigateToPoseアクションへ。send_goal.sh等の互換用 ---
    goal_pose_bridge = Node(
        package='go2_path_following',
        executable='goal_pose_bridge',
        name='goal_pose_bridge',
        output='screen',
        parameters=[NO_SIM_TIME],
    )

    # --- 接触検知(#23派生): LiDAR近距離+IMU/オドメトリ不一致でcollision_detectedを
    #     発行し、bt_navigatorがprogress_checkerの10秒待ちより速くリカバリを始める ---
    #     IMU積分・LiDARタイミングはsim時刻に同期させる必要がある
    collision_detector = Node(
        package='go2_path_following',
        executable='collision_detector',
        name='collision_detector',
        output='screen',
        parameters=[SIM_TIME],
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
        DeclareLaunchArgument('use_following', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        localization,
        planner,
        following,
        bt_navigator,
        behavior_server,
        lifecycle_manager_bt,
        goal_pose_bridge,
        collision_detector,
        cmd_vel_safety,
        tf_relay,
        rviz,
    ])
