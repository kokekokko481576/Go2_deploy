# go2_bt_plugins — 経路追従BT(Issue #50)

`nav2_bt_navigator`用のカスタムBehaviorTree.CPPプラグインと、経路追従+リカバリ用の
BT XMLを置くパッケージ。

## 背景

追M3(#23)で接触検知(`collision_detector`)→リカバリ(その場回転→後退→再計画)を
`plan_follower`に手書きの状態機械として実装したが、条件分岐・タイマー・リトライ状態が
積もってきた。「判断そのものをブラックボックス化したくない」という方針から、
判断のオーケストレーションを`nav2_bt_navigator`(BehaviorTree.CPP)へ移した。

BT担当者は別途今年の計画内(跨ぎスキル切り替え等、Issue #25)で着手予定だが、
その前段となる経路追従レイヤの雛形として先行実装した。

## 構成

- `include/go2_bt_plugins/is_collision_detected_condition.hpp` /
  `src/is_collision_detected_condition.cpp`: `collision_detector`(#23、
  LiDAR近距離+IMU/オドメトリ不一致)の`/collision_detected`(`std_msgs/Bool`)を
  見るBT条件ノード。`nav2_behavior_tree::IsBatteryLowCondition`と同じ構成
  (トピックsubscribe→SUCCESS/FAILURE)。XML上のタグ名は`IsCollisionDetected`
- `behavior_trees/navigate_with_collision_recovery.xml`: stockの
  `navigate_to_pose_w_replanning_and_recovery.xml`(nav2_bt_navigator付属)を土台に、
  `FollowPath`を`ReactiveSequence`+`Inverter(IsCollisionDetected)`でラップした

```xml
<RecoveryNode number_of_retries="1" name="FollowPath">
  <ReactiveSequence name="FollowPathUnlessCollision">
    <Inverter>
      <IsCollisionDetected topic_name="/collision_detected"/>
    </Inverter>
    <FollowPath path="{path}" controller_id="FollowPath"/>
  </ReactiveSequence>
  <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
</RecoveryNode>
```

`collision_detected`がtrueになると`IsCollisionDetected`がSUCCESS→`Inverter`で
FAILUREに反転→`ReactiveSequence`全体がFAILURE→実行中の`FollowPath`が即halt
(キャンセル)される。これにより`progress_checker`の10秒タイムアウトを待たず、
外側の`RecoveryNode`(`NavigateRecovery`、stockのまま)がSpin/Wait/BackUp
(`RoundRobin`で順に試す)のリカバリへ即座に落ちる。

## ビルド時の注意(BehaviorTree.CPP v3のプラグイン登録)

- カスタムノードの`.cpp`末尾で`#define BT_PLUGIN_EXPORT`を`behaviortree_cpp_v3/bt_factory.h`
  のincludeより**前**に置く必要がある。無いと`BT_REGISTER_NODES`マクロが
  `BTCPP_EXPORT`を`static`に展開してしまい、共有ライブラリから
  `BT_RegisterNodesFromPlugin`シンボルがexportされず、`bt_navigator`の
  `configure`が`Could not load library`で失敗する(実際に一度踏んだ)
- `ReactiveSequence`/`Inverter`/`Sequence`/`Fallback`等はBehaviorTree.CPPコア
  組み込みノードなので、`plugin_lib_names`への追加は不要(誤って追加すると
  存在しないライブラリ名でロードエラーになる)
- `RecoveryNode`/`PipelineSequence`/`RoundRobin`/`ComputePathToPose`/`FollowPath`/
  `Spin`/`Wait`/`BackUp`/`ClearEntireCostmap`/`GoalUpdated`等は`nav2_behavior_tree`/
  `nav2_behaviors`側のプラグイン(`.so`)なので、`bt_navigator.yaml`の
  `plugin_lib_names`に列挙する必要がある(`go2_path_following/config/bt_navigator.yaml`)
- `bt_navigator`は`default_nav_to_pose_bt_xml`と`default_nav_through_poses_bt_xml`の
  **両方**をconfigure時に検証する。`NavigateThroughPoses`は使わないが、指定しないと
  stockの`navigate_through_poses_w_replanning_and_recovery.xml`が読まれ、
  そちらが要求する追加プラグイン(`nav2_remove_passed_goals_action_bt_node`等)が
  無くて設定失敗する。両方に同じXMLパスを渡すことで回避した

## behavior_server(nav2_behaviors)のパラメータの注意

- `Spin`は`max_rotational_vel`/`min_rotational_vel`/`rotational_acc_lim`を
  **フラットな**トップレベルパラメータとして持つ
- `BackUp`(`DriveOnHeading`テンプレート)は`<behavior_plugins名>.acceleration_limit`/
  `<behavior_plugins名>.deceleration_limit`/`<behavior_plugins名>.minimum_speed`と
  **behavior名でprefixされた**パラメータを持つ(Spinとは命名規則が違う。
  未設定だと加減速リミット無しでDriveOnHeadingがタイムアウトして失敗する)
- `Spin`自身も局所コストマップに対する衝突チェックを内部で持っており、接触直後で
  コストマップに障害物が残っている状況では回転そのものを拒否してタイムアウトする
  ことがある(Gazeboで実際に確認。バグではなく意図された安全動作)。その場合は
  `RoundRobin`が次のリカバリ手段(`Wait`等)に自動的にフォールバックする

## Gazeboでの動作確認(2026-08-04)

sim再起動直後(クリーンな状態)から接触を誘発するゴールを1回投入し、以下の一連を確認:

1. `collision_detector`がIMU/オドメトリ不一致を検知
2. `FollowPath`が即halt(`ReactiveSequence`によるキャンセル)
3. `RoundRobin`が`Spin`を選択 → 局所コストマップ起因で失敗(上記の安全動作)
4. `RoundRobin`が`Wait`にフォールバック → 成功
5. 外側`RecoveryNode`が`ComputePathToPose`からやり直し → 追従再開 → ゴール到達

真値(Gazebo物理、`gz topic -e -t /world/default/pose/info`)との比較では、
この1回の接触で位置に約0.49mのオドメトリズレが残った(向きは4.4°差とほぼ正確)。
接触自体を無くす・ズレを縮める対策はIssue #14側の課題。

## 未実施 / 今後

- MPPI(#22)との切替: 現在のXMLは`controller_id="FollowPath"`固定。別XML or
  値の外出しが必要
- 「後退直後の即再接触は3回粘らず断念する」という#23独自のバックストップは、
  `RecoveryNode`標準のリトライ回数制限に簡略化された(時間窓の判定は落ちた)
- `BehaviorTreeEngine: tick rate 100.00 was exceeded`警告の原因調査
