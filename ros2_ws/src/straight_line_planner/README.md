# straight_line_planner

経路生成計画 Phase1 M1(`docs/計画/経路生成.md` §3.2)。自己位置から目標作業姿勢まで
直線補間した `nav_msgs/Path` を出すだけの最小プランナ。Phase2で Nav2 の planner server に
差し替えても経路追従側のI/Fが変わらないよう、トピック名・型をNav2に合わせている。

## I/F

| 種別 | トピック | 型 | 備考 |
|------|---------|-----|------|
| 購読 | `goal_pose` | `geometry_msgs/PoseStamped` | 目標作業姿勢(部材への正対点)。Phase1では手打ち・手動publish可 |
| 配信 | `plan` | `nav_msgs/Path` | Nav2 planner_serverの既定出力トピック名に合わせている |
| TF   | `map` → `base_link` | — | 自己位置。`global_frame`/`robot_base_frame`パラメータで変更可 |

パラメータ: `global_frame`(既定`map`)、`robot_base_frame`(既定`base_link`)、
`path_resolution`(既定0.1m、経路点の間隔)。

中間点の姿勢は進行方向を向かせ、終端のみゴールの姿勢(部材への正対)をそのまま使う。

## 使い方

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select straight_line_planner
source install/setup.bash
ros2 run straight_line_planner straight_line_planner_node
```

別ターミナルでゴールをpublish(TFが `map`→`base_link` で通っている前提):

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 6.0}, orientation: {z: 0.7071, w: 0.7071}}}" --once
ros2 topic echo /plan --once
```

## 動作確認結果(2026-07-12)

`docker/`(dev)コンテナ内で、`tf2_ros static_transform_publisher` で `map`→`base_link` を
固定配信した状態(実機・Gazeboなしのシミュレーション相当)で確認:

- goal_pose受信 → 距離に応じた点数(0.1m間隔)で `plan` を配信することを確認
- 中間点の向きが進行方向(atan2)になっていること、終端がゴールの姿勢と一致することを確認

経路追従班(Nav2コントローラ)との統合は2026-07-14に確認済み(`go2_path_following`の
`plan_follower`が本ノードの`plan`を購読しGazebo上でゴール到達まで確認。詳細は
`go2_path_following/README.md`・Issue #21)。

未実施:

- Gazebo・実機での部材正対精度・到達成功率の**定量**計測(§3.5の完了条件。上記統合確認は
  定性的な到達確認までで、精度の数値評価はGATE1のベースライン化と合わせてこれから)
