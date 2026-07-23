# go2_path_following — 経路追従（Nav2 controller_server / 現状DWB・MPPIは#22で比較予定）

経路追従 練習(追M2: 平地の経路追従)用のbringupパッケージ。`go2_localization`と同様、
自作ノードは`plan_follower`(小さな橋渡し)のみで、本体は既製の`nav2_controller`と
その設定ファイル+起動ファイルで構成する。現在有効なローカルプランナは
**DWB**(`dwb_core::DWBLocalPlanner`)で、MPPIとの比較(Issue #22)は未実施。

---

## 使い方(フェーズB、現在の既定)

devコンテナ + simコンテナの両方を起動した状態で、まずビルドする:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_path_following
source install/setup.bash
```

続いて、別ターミナルで以下を順に起動する(フェーズBは`go2_localization`の起動が前提。
フェーズAで確認したい場合は`controller.launch.py`の`/tf` remapを一時的に`/robot1/tf`へ
戻す。フェーズA/Bの意味は「概要」参照):

```bash
# 1) 自己位置推定(EKF/AMCL) … 自作TF /go2_localization/tf を配信
ros2 launch go2_localization localization.launch.py

# 2) controller_server + lifecycle_manager … このパッケージの本体
ros2 launch go2_path_following controller.launch.py

# 3) 橋渡しノード … plan トピックを FollowPath ゴールに変換
ros2 run go2_path_following plan_follower

# 4) 経路生成 … Path(/plan)を配信。フェーズBは自作TFへのremapが必須
#    (素の /tf には誰も配信していないため、忘れるとPathが一切出ない)
ros2 run straight_line_planner straight_line_planner_node --ros-args -p use_sim_time:=true \
  -r /tf:=/go2_localization/tf

# 5) 速度セーフティ … controller の生出力を最終駆動トピックへ中継
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args -r cmd_vel:=/robot1/cmd_vel
```

### ゴールを投入して動作確認

ゴールを`/goal_pose`にpublishすると、`straight_line_planner`がPathを生成し、
`plan_follower`経由でロボットが追従を始める。YAML手打ちはスペース位置を間違えやすいので
`scripts/send_goal.sh`を推奨(`x` `y` `yaw(度)`を引数で渡す。省略時は`x=3.0 y=0.0 yaw=0`):

```bash
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh 3.0 0.0     # 3m前方へ
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh 2.0 1.5 90  # 斜め先で左向きに正対

# スクリプトの中身は以下と等価(-w 1でマッチ待ち・-t 3で3回送信=配送遅延対策 #7):
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}" \
  -w 1 -t 3 -r 1
```

Gazebo上のGo2が実際に前進すれば配線は成功。RViz(sim側で自動起動するもの)で`/plan`と
ロボットの動きを見比べるとわかりやすい。

### 無反応のときの切り分け

ゴールをpublishしてもロボットが動かないときは、まず経路が出ているかを確認する:

```bash
# Path(/plan)が出ているか。出ていれば上流(planner)はOK → 下流(controller/TF)を疑う
ros2 topic echo /plan --qos-durability transient_local --once

# 最終駆動トピックのpublisherを確認(下記「詳細」のトピック表も参照)
ros2 topic info /robot1/cmd_vel -v
```

- `/plan`が出ない → `straight_line_planner_node`の`/tf` remap忘れ、または`use_sim_time`
  未指定(下記「よくある詰まり」参照)。
- `/plan`は出るのにロボットが動かない → `plan_follower`/`controller_server`/
  `cmd_vel_safety`のいずれかの配線、またはlifecycle未起動を疑う。

---

## 概要

### このサブシステムの位置づけ

自律移動の3計画(自己位置推定・経路生成・経路追従)のうち、本パッケージは
**経路追従**を担う。計画書のマイルストーンとの対応は以下:

- **M1(テレオペ`cmd_vel`確認)**: 手動指令でロボットが期待どおり動くことの確認。
  本パッケージの最終段は`cmd_vel_safety`を経て`/robot1/cmd_vel`へ出す構成で、
  この駆動経路がM1と共通。
- **M2(controller server / 追M2: 平地の経路追従・Issue #21)**: `straight_line_planner`が
  出した`Path`を`controller_server`(DWB)で追従し、`cmd_vel`を生成する。本パッケージの
  主目的。障害物回避(ローカルコストマップ)は完了条件に含めず、追M3(Issue #23)で扱う。

### なぜこの設計になっているか(bt_navigatorを使わない理由)

Nav2の`controller_server`は`FollowPath`アクション(`nav2_msgs/action/FollowPath`)で
駆動する仕組みで、`nav_msgs/Path`トピックを直接subscribeするわけではない。本来は
`bt_navigator`がこのアクションを呼ぶが、本パッケージは`bt_navigator`を導入せず
「`straight_line_planner`が生成済みのPathを追従させてみる」(Issue #21)という最小構成に
とどめる。そのため`plan_follower`ノードが`plan`トピックを購読し、受け取るたびに
`FollowPath`ゴールとして送るだけの橋渡しを行う(リカバリ・再計画はM3以降)。

`controller_server`はGATE1で1つだけ動かす前提のため、simコンテナ(upstream本家)が既に
動かしている`controller_server`とはノード名・トピック名が別になるよう配線している
(`cmd_vel`は直接simへ出さず`/cmd_vel_raw`→`cmd_vel_safety`経由にする等)。
ただし**最終的な駆動トピック`/robot1/cmd_vel`だけは分離できない**(simの`cmd_vel_pub`が
唯一の歩容入力として購読するため)。upstream本家のNav2側も`controller_server`(1個)と
`behavior_server`(behaviorプラグイン分5個)が`/robot1/cmd_vel`へのpublisherを持つことを
実測で確認済み(Issue #35、2026-07-17)。汚染防止のため、GATE1計測時はsimを
`enable_nav2:=false`で起動してupstream Nav2を丸ごと止める(下記「GATE1計測時の…」)。

### データフロー

```
/goal_pose ──▶ straight_line_planner ──/plan(Path)──▶ plan_follower
                                                            │
                                          follow_path(FollowPath action)
                                                            ▼
        /go2_localization/tf ──(現在位置)──▶ controller_server(DWB)
                                                            │
                                                  /cmd_vel_raw(Twist)
                                                            ▼
                                                     cmd_vel_safety
                                                            │
                                                   /robot1/cmd_vel
                                                            ▼
                                                    sim(cmd_vel_pub→歩容)
```

### ノード

| ノード | 実行元 | 役割 |
|--------|--------|------|
| `controller_server` | `controller.launch.py`(`nav2_controller`) | Pathを追従して`cmd_vel`を生成。本体 |
| `lifecycle_manager_navigation` | `controller.launch.py`(`nav2_lifecycle_manager`) | `controller_server`のlifecycleを`autostart` |
| `plan_follower` | `ros2 run`(このパッケージ) | `/plan`を`FollowPath`ゴールへ橋渡し |

### 入出力(トピック / アクション)

| 名前 | 型 | 向き | ノード | 備考 |
|------|----|------|--------|------|
| `/plan` | `nav_msgs/msg/Path` | 購読 | `plan_follower` | `straight_line_planner`が配信。QoSは`transient_local` |
| `follow_path` | `nav2_msgs/action/FollowPath` | クライアント→サーバ | `plan_follower`→`controller_server` | `goal.controller_id`は既定`FollowPath` |
| `/go2_localization/tf` | `tf2_msgs/msg/TFMessage` | 参照 | `controller_server` | フェーズBの`map→odom→base_link`。launchで`/tf`をremap |
| `/robot1/tf_static` | `tf2_msgs/msg/TFMessage` | 参照 | `controller_server` | launchで`/tf_static`をremap |
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 発行 | `controller_server` | DWBの生出力(launchで`cmd_vel`をremap) |
| `/robot1/cmd_vel` | `geometry_msgs/msg/Twist` | 発行 | `cmd_vel_safety` | 最終駆動指令(simが購読する唯一の入力) |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | (上流) | `straight_line_planner`が購読 | `send_goal.sh`で投入 |

---

## 詳細

### plan_follower の内部ロジック

`go2_path_following/plan_follower.py`。ノード名`plan_follower`。動作は単純で、以下だけを行う:

1. パラメータ`controller_id`(既定`FollowPath`)を読み、`follow_path`アクションの
   `ActionClient`と、`plan`トピック(`nav_msgs/Path`、キュー長10)の購読を作る。
2. `plan`を受信するたび`on_plan()`が走る:
   - `follow_path`サーバを1秒待って不在なら警告(5秒throttle)して`return`。
   - **実行中のゴールがあればキャンセル**(`cancel_goal_async`)してから、
   - 新しい`Path`を`FollowPath.Goal`に詰め、`controller_id`を設定して非同期送信する。
3. ゴール応答コールバック`_on_goal_response()`で、accept時のみ`_goal_handle`を保持
   (reject時は警告してハンドルを持たない)。

つまり「新しいPathが来たら古い追従を止めて送り直す」だけで、フィードバック監視・
結果待ち・リトライは持たない。終了判定(ゴール到達)は`controller_server`側の
`goal_checker`(後述)が担い、`plan_follower`はそれを監視しない。

### controller.launch.py の配線

`launch/controller.launch.py`が2ノードを起動する:

- **`controller_server`**(`nav2_controller`) — パラメータは`config/controller_server.yaml`。
  remap:
  - `cmd_vel` → `/cmd_vel_raw`(生出力を最終駆動から分離)
  - `/tf` → `/go2_localization/tf`(フェーズB。フェーズAでは`/robot1/tf`を参照していた)
  - `/tf_static` → `/robot1/tf_static`
- **`lifecycle_manager_navigation`**(`nav2_lifecycle_manager`) — 同じYAMLを読み、
  `autostart: true`で`node_names: ["controller_server"]`をconfigure/activateする。

### controller_server.yaml の主要パラメータ

`external/go2_ros2_sim_py/gazebo_sim/config/nav2_params.yaml`のDWBブロックを参考値として
流用したもの。

#### controller_server 本体・チェッカ

| パラメータ | 値 | 意味 |
|-----------|----|------|
| `use_sim_time` | `true` | sim連携のため必須 |
| `controller_frequency` | `20.0` | 制御ループ周波数[Hz] |
| `min_x_velocity_threshold` | `0.001` | この速度未満は0扱い(x) |
| `min_y_velocity_threshold` | `0.5` | 同(y)。四脚は横移動を使わないため高め |
| `min_theta_velocity_threshold` | `0.001` | 同(θ) |
| `progress_checker` | `SimpleProgressChecker` | `required_movement_radius: 0.5` / `movement_time_allowance: 10.0`(10秒で0.5m進まないと停滞と判定) |
| `general_goal_checker` | `SimpleGoalChecker` | `stateful: true` / `xy_goal_tolerance: 0.25` / `yaw_goal_tolerance: 0.2`(到達判定の許容誤差) |
| `controller_plugins` | `["FollowPath"]` | 使うローカルプランナ名 |

#### FollowPath(DWBLocalPlanner)

| パラメータ | 値 | 意味 |
|-----------|----|------|
| `plugin` | `dwb_core::DWBLocalPlanner` | ローカルプランナ本体(**現状DWB**、MPPIは#22で比較予定) |
| `max_vel_x` / `max_speed_xy` | `0.18` | 前進速度上限[m/s](Go2は控えめに設定) |
| `min_vel_x` | `0.0` | 後退させない |
| `max_vel_y` / `max_vel_theta` | `0.0` / `0.5` | 横速度なし / 旋回上限[rad/s] |
| `acc_lim_x` / `acc_lim_theta` | `2.5` / `3.2` | 加速度上限。減速側は`decel_lim_* = -2.5 / -3.2` |
| `vx_samples` / `vy_samples` / `vtheta_samples` | `20` / `5` / `20` | 速度サンプリング数 |
| `sim_time` | `1.7` | 各候補軌道の前方シミュレーション時間[s] |
| `linear_granularity` / `angular_granularity` | `0.05` / `0.025` | 軌道評価の刻み |
| `transform_tolerance` | `0.2` | TF許容遅延[s] |
| `xy_goal_tolerance` | `0.25` | ゴール到達の距離許容[m] |
| `critics` | 下記7個 | 軌道の評価関数群 |

critics(評価関数)とscale:

| critic | scale | 役割 |
|--------|-------|------|
| `PathAlign` | `32.0` | 経路への姿勢整合(`forward_point_distance: 0.1`) |
| `PathDist` | `32.0` | 経路からの距離 |
| `GoalAlign` | `24.0` | ゴールへの姿勢整合(`forward_point_distance: 0.1`) |
| `GoalDist` | `24.0` | ゴールまでの距離 |
| `RotateToGoal` | `32.0` | ゴール姿勢への旋回(`slowing_factor: 5.0` / `lookahead_time: -1.0`) |
| `Oscillation` | (既定) | 振動抑制 |
| `BaseObstacle` | `0.02` | 障害物回避(M2ではコストマップが最小構成のため寄与は小) |

#### local_costmap

M2(平地の経路追従)は障害物回避を完了条件に含めない(計画書M3で扱う)ため、
**`inflation_layer`のみの最小構成**。`static_layer`/`voxel_layer`は追M3(Issue #23)で追加する。

| パラメータ | 値 |
|-----------|----|
| `global_frame` / `robot_base_frame` | `odom` / `base_link` |
| `rolling_window` | `true`(ロボット追従の移動窓) |
| `width` × `height` / `resolution` | `10` × `10` [m] / `0.05` [m/cell] |
| `update_frequency` / `publish_frequency` | `5.0` / `2.0` [Hz] |
| `footprint` | `[[0.35,0.18],[0.35,-0.18],[-0.38,-0.18],[-0.38,0.18]]`(Go2の外形) |
| `plugins` | `["inflation_layer"]`(`cost_scaling_factor: 4.0` / `inflation_radius: 0.45`) |

#### lifecycle_manager_navigation

`autostart: true` / `node_names: ["controller_server"]`。起動時に`controller_server`を
自動でactivateする。

### フェーズA/B(TFの参照先)

`controller_server`は実際の`map→odom→base_link`のTFで現在位置を把握する。この参照先を
2段階で切り替える:

- **フェーズA**: upstream(sim本家)が配信する`/robot1/tf`をそのまま参照する。
  `go2_localization`側の変更なしに`controller_server`・`plan_follower`・`cmd_vel_safety`の
  配線自体が正しく動くかを先に確定させる、動作確認専用の段階
  (戻すには`controller.launch.py`の`/tf` remapを`/robot1/tf`へ書き換える)。
- **フェーズB(現在の既定)**: `go2_localization`のEKF/AMCLのTF配信を有効化した後、
  `controller.launch.py`の`/tf` remapを`/go2_localization/tf`にして、自作の自己位置推定の
  TFで経路追従する。両フェーズともGazeboでの到達確認済み(下記)。

### GATE1計測時のトピック確認(Issue #35)

GATE1の定量計測(到達成功率・部材正対精度・ゴール姿勢誤差)では、upstream本家のNav2
スタックが同じ`/robot1/cmd_vel`にpublisherを持ったまま並走していると計測を汚染しうるため、
「自作パイプラインだけがロボットを駆動している」ことを確認してから計測する。

1. **simをupstream Nav2なしで起動する**(fork一次カスタマイズで追加した`enable_nav2`引数。
   composeが`SIM_ENABLE_NAV2`環境変数でlaunch引数に変換する):

   ```bash
   cd docker/sim
   SIM_ENABLE_NAV2=false SIM_ENABLE_RVIZ=false docker compose up -d
   # 戻すときは普通に: docker compose up -d (従来のNav2入りで再作成される)
   ```

   これで`map_server`/`amcl`/`planner`/`controller`/`behavior`/`smoother`/`bt_navigator`が
   一切起動しない(ロボットspawn・歩容・odom・EKF・センサブリッジは従来どおり)。
   なおフェーズA(upstreamの`/robot1/tf`参照)はamclが止まるため使えないが、
   GATE1はフェーズB前提なので支障ない。

2. **`/robot1/cmd_vel`のpublisherを確認する**(simコンテナ内で):

   ```bash
   ros2 topic info /robot1/cmd_vel -v
   ```

   - `enable_nav2:=false`で自作パイプライン起動前: publisherは**0個**のはず
     (従来はupstreamの`controller_server`1個+`behavior_server`5個=6個が常駐)。
   - `cmd_vel_safety_node`(`-r cmd_vel:=/robot1/cmd_vel`)起動後: publisherは**1個だけ**のはず。
   - ノード名が`_NODE_NAME_UNKNOWN_`と表示される場合(Issue #7)は
     `ros2 node info --no-daemon /robot1/controller_server`等、`--no-daemon`付きのノード側からの
     確認で代替できる(daemonのグラフキャッシュがHumble⇔Jazzy混在で壊れているだけで、
     通信自体は正常)。

3. **計測でみるトピックの整理**:

   | トピック | 中身 | 用途 |
   |---------|------|------|
   | `/cmd_vel_raw` | 自作`controller_server`(DWB)の生出力 | 追従指令そのものの記録 |
   | `/robot1/cmd_vel` | `cmd_vel_safety`経由の最終駆動指令 | ロボットに実際に届く指令の記録 |
   | `/go2_localization/tf` | 自作EKF/AMCLの`map→odom→base_link` | 到達判定・ゴール姿勢誤差の算出 |
   | `/goal_pose` | 計測試行ごとのゴール | 試行の開始トリガ・正解値 |

   自作パイプラインの「実出力」は`/cmd_vel_raw`(launch内で`cmd_vel`をremap済み)であり、
   `/robot1/cmd_vel`はあくまで`cmd_vel_safety_node`の手動remapを経た後の最終段である点に注意。

### 動作確認結果(2026-07-14、Issue #21)

フェーズA(upstream本家の`/robot1/tf`)・フェーズB(自作EKF/AMCLの`/go2_localization/tf`、
`go2_localization`のTF配信を有効化した後)の両方で、上記手順どおりにGazebo上のGo2が
実際にゴールまで到達することを確認した。

### よくある詰まり

- **`use_sim_time`忘れ**(2026-07-14): `straight_line_planner_node`を`ros2 run`で単体起動する際に
  `use_sim_time`の指定を忘れ、壁時計でPathをスタンプしていたため`controller_server`側で
  `Transform data too old when converting from map to odom`が出続けて`Failed to make progress`に
  なった(sim連携ノードは`use_sim_time:=true`の明示が必須)。
- **`/tf` remap忘れ**(2026-07-17): `straight_line_planner_node`の`/tf` remapを忘れると、ゴールを
  publishしてもTF lookupに失敗してPathが一切出ない(=ロボットが動かない)。本ノードは素の`/tf`を
  購読するが、フェーズB構成で素の`/tf`に配信するノードは存在しない(自作TFは
  `/go2_localization/tf`、simは`/robot1/tf`)。無反応時はまず`ros2 topic echo /plan
  --qos-durability transient_local --once`でPathが出ているかを見ると上流/下流を切り分けられる。

### 未実施 / 今後

- MPPIとの比較(Issue #22)。現状のローカルプランナはDWBのみ。
- ローカルコストマップでの障害物回避(Issue #23、追M3)。`static_layer`/`voxel_layer`の追加。
- ゴール姿勢誤差の定量評価(計画書M2の完了条件の精密な検証。GATE1の到達成功率・
  部材正対精度のベースライン化もここに含む)。
