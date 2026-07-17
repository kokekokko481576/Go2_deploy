# go2_path_planning

経路生成 練習(生M2: コストマップ+グリッド探索、Issue #16/#17)用のbringupパッケージ。
`go2_path_following`と同じ流儀で、自作ノードは`plan_requester`(小さな橋渡し)のみ、
本体は既製の`nav2_planner`(NavFn=**ダイクстラ法**)の設定ファイル+起動ファイルで構成する。

## なぜこの設計になっているか

Nav2の`planner_server`は`ComputePathToPose`アクションで駆動する仕組みで、
`goal_pose`トピックを直接subscribeするわけではない。本来は`bt_navigator`がこの
アクションを呼ぶが、`plan_follower`(経路追従側)と同様に`bt_navigator`は導入せず、
`plan_requester`が`goal_pose`を購読→アクション要求→結果のPathを`plan`へ流す
最小構成にとどめる。

**`straight_line_planner`(生M1)とトピックI/F(goal_pose/plan、QoS含む)を完全に
揃えてある**ため、下流(`plan_follower`→`controller_server`→`cmd_vel_safety`)は
どちらのプランナが動いているか気づかない。差し替え=「どちらを起動するか」だけ。

プランナはNavFnの`use_astar: false`(=ダイクストラ)。コストマップは計画書
(`docs/計画/経路生成.md` §3.3)どおり static layer(既知地図) + inflation layer の2層で、
obstacle layer(顎LiDARライブスキャン)は生M3(Issue #18)で追加する。footprintは
`go2_path_following`のlocal_costmapと同じ共有定義(C5)。地図は自作`map_server`の
`/go2_localization/map`を購読する(upstreamと分離)。

## 使い方(フェーズB構成)

devコンテナ + simコンテナの両方を起動した状態で:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_path_planning
source install/setup.bash

# 別ターミナルでそれぞれ起動(straight_line_plannerの代わりに下2つを使う。
# それ以外はgo2_path_following/README.mdの「使い方(フェーズB)」と同じ)
ros2 launch go2_localization localization.launch.py
ros2 launch go2_path_planning planner.launch.py
ros2 run go2_path_planning plan_requester
ros2 launch go2_path_following controller.launch.py
ros2 run go2_path_following plan_follower
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args -r cmd_vel:=/robot1/cmd_vel
```

ゴール投入は従来どおり:

```bash
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh -1.71 4.70 90
```

## 動作確認結果(2026-07-17、Issue #16/#17)

cafeワールド(Gazebo)・フェーズB(自作TF)で確認:

- **#16(最小起動)**: planner_server+global_costmap(static+inflation)がアクティブ化し、
  ゴール(2.5, 3.0)に対し156点のPathを生成(計画時間0.001s)。障害物のない方向では
  経路長3.92m vs 直線3.91mでほぼ直線=妥当
- **#17(既知障害物回避)**: 直線が地図上の障害物(-1.2, 3.4付近)を横切るゴール
  (-1.71, 4.70)に対し、経路長5.32m vs 直線5.00m(+0.32m)の**迂回経路**を生成。
  そのままフルチェーン(plan_follower→DWB→cmd_vel_safety)でロボットが実際に
  迂回歩行し`Reached the goal!`(最終誤差約0.18m、トレランス0.25m内)を確認

## 詰まりどころ(既知)

- **`/goal_pose`の配送が間欠的に遅延する**ことがある(Humble⇔Jazzy混在環境、#7の
  周辺症状と思われる)。`ros2 topic pub --once`は論外(マッチング前publishで消える)、
  `-w 1 -t 2`でも届かないことがあり、後続の送信でまとめて届く挙動を観測した。
  `send_goal.sh`は複数回送信にしてあるが、無反応なら
  `ros2 topic echo /plan --qos-durability transient_local --once`でPathの有無を見て
  もう一度送るのが早い(`plan_requester`のログに`published path`が出れば計画は済んでいる)
- planner_serverが`計画結果が空`を返す場合: ゴールが障害物内(インフレーション込み)か
  コストマップ外。ゴール位置を少しずらす

## 未実施

- MPPI/Smac系との比較(まずはNavFnダイクストラを既定とする)
- obstacle layer追加(生M3、Issue #18)・再計画のトリガ設計(経路無効化時)
- 経路品質の定量評価(障害物配置を変えた成功率・経路長・計画時間。生M2完了条件の
  「シミュレーション→実機」の実機側も未実施)
