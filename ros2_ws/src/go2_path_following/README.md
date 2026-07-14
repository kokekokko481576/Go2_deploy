# go2_path_following

経路追従 練習(追M2: 平地の経路追従)用のbringupパッケージ。`go2_localization`と同様、
自作ノードは`plan_follower`(小さな橋渡し)のみで、本体は既製の`nav2_controller`
(DWBLocalPlanner)の設定ファイル+起動ファイルで構成する。

## なぜこの設計になっているか

Nav2の`controller_server`は`FollowPath`アクション(`nav2_msgs/action/FollowPath`)で
駆動する仕組みで、`nav_msgs/Path`のトピックを直接subscribeするわけではない。本来は
`bt_navigator`がこのアクションを呼ぶが、このパッケージは`bt_navigator`を導入せず
「`straight_line_planner`が生成済みのPathを追従させてみる」(Issue #21)という
最小構成にとどめる。そのため`plan_follower`ノードが`plan`トピックを購読し、
受け取るたびに`FollowPath`ゴールとして送るだけの橋渡しを行う。

このパッケージは`controller_server`をGATE1で1つだけ動かす前提のため、simコンテナ
(upstream本家)が既に動かしている`controller_server`とはノード名・トピック名が
別になるよう(`cmd_vel`は直接simへ出さず`cmd_vel_safety`経由にする等)配線している。

## フェーズA/B(TFの参照先)

`controller_server`は実際の`map→odom→base_link`のTFで現在位置を把握する。このTFの
参照先を2段階で切り替える:

- **フェーズA(現在の既定)**: upstream(sim本家)が既に配信している`/robot1/tf`を
  そのまま参照する。`go2_localization`側の変更なしに`controller_server`・
  `plan_follower`・`cmd_vel_safety`の配線自体が正しく動くかを先に確定させる
- **フェーズB**: `go2_localization`のEKF/AMCLのTF配信を有効化した後、
  `launch/controller.launch.py`の`('/tf', '/robot1/tf')`を
  `('/tf', '/go2_localization/tf')`に変更し、自作の自己位置推定のTFで
  経路追従できることを確認する

## 使い方(フェーズA)

devコンテナ + simコンテナの両方を起動した状態で:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_path_following
source install/setup.bash

# 別ターミナルでそれぞれ起動
ros2 launch go2_path_following controller.launch.py
ros2 run go2_path_following plan_follower
ros2 run straight_line_planner straight_line_planner_node
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args -r cmd_vel:=/robot1/cmd_vel
```

ゴールをpublishして実際に動くか確認:

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}" --once
```

Gazebo上のGo2が実際に前進すれば配線は成功。RViz(sim側自動起動のもの)で`plan`と
ロボットの動きを見比べるとわかりやすい。

## 動作確認結果

まだ未実施。
