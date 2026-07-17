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

ただし**最終的な駆動トピック`/robot1/cmd_vel`だけは分離できない**(simの`cmd_vel_pub`が
唯一の歩容入力として購読するため)。upstream本家のNav2側も`controller_server`(1個)と
`behavior_server`(behaviorプラグイン分5個)が`/robot1/cmd_vel`へのpublisherを持つことを
実測で確認済み(Issue #35、2026-07-17)。upstreamのNav2にゴールを与えない限り実際の
Twistは流れてこないが、GATE1の定量計測時は汚染防止のためsim側を`enable_nav2:=false`で
起動してupstream Nav2を丸ごと止める(下記「GATE1計測時のトピック確認」)。

## フェーズA/B(TFの参照先)

`controller_server`は実際の`map→odom→base_link`のTFで現在位置を把握する。このTFの
参照先を2段階で切り替える:

- **フェーズA**: upstream(sim本家)が既に配信している`/robot1/tf`をそのまま参照する。
  `go2_localization`側の変更なしに`controller_server`・`plan_follower`・
  `cmd_vel_safety`の配線自体が正しく動くかを先に確定させるための、動作確認専用の段階
  (現在は`launch/controller.launch.py`を書き換えないと戻せない)
- **フェーズB(現在の既定)**: `go2_localization`のEKF/AMCLのTF配信を有効化した後、
  `launch/controller.launch.py`の`/tf`remapを`/go2_localization/tf`にして、
  自作の自己位置推定のTFで経路追従する。両フェーズともGazeboでの到達確認済み(下記)

## 使い方(フェーズB、現在の既定)

devコンテナ + simコンテナの両方を起動した状態で:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_path_following
source install/setup.bash

# 別ターミナルでそれぞれ起動(フェーズBはgo2_localizationの起動が前提。
# フェーズAで確認したい場合はcontroller.launch.pyの/tf remapを一時的に/robot1/tfへ戻す)
ros2 launch go2_localization localization.launch.py
ros2 launch go2_path_following controller.launch.py
ros2 run go2_path_following plan_follower
ros2 run straight_line_planner straight_line_planner_node --ros-args -p use_sim_time:=true
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args -r cmd_vel:=/robot1/cmd_vel
```

ゴールをpublishして実際に動くか確認:

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}" --once
```

Gazebo上のGo2が実際に前進すれば配線は成功。RViz(sim側自動起動のもの)で`plan`と
ロボットの動きを見比べるとわかりやすい。

## GATE1計測時のトピック確認(Issue #35)

GATE1の定量計測(到達成功率・部材正対精度・ゴール姿勢誤差)では、upstream本家の
Nav2スタックが同じ`/robot1/cmd_vel`にpublisherを持ったまま並走していると計測を
汚染しうるため、以下の手順で「自作パイプラインだけがロボットを駆動している」ことを
確認してから計測する。

1. **simをupstream Nav2なしで起動する**(fork一次カスタマイズで追加した`enable_nav2`引数):

   ```bash
   # docker/sim/compose.yaml のcommandを一時的に差し替えるか、コンテナ内で直接:
   ros2 launch gazebo_sim launch.py use_sim_time:=true enable_nav2:=false
   ```

   これでmap_server/amcl/planner/controller/behavior/smoother/bt_navigatorが
   一切起動しない(ロボットspawn・歩容・odom・EKF・センサブリッジは従来どおり)。
   なおフェーズA(upstreamの`/robot1/tf`参照)はamclが止まるため使えない。
   GATE1はフェーズB(自作`/go2_localization/tf`)前提なので支障ない。

2. **`/robot1/cmd_vel`のpublisherを確認する**(simコンテナ内で):

   ```bash
   ros2 topic info /robot1/cmd_vel -v
   ```

   - `enable_nav2:=false`で自作パイプライン起動前: publisherは**0個**のはず
     (従来はupstreamの`controller_server`1個+`behavior_server`5個=6個が常駐)
   - `cmd_vel_safety_node`(`-r cmd_vel:=/robot1/cmd_vel`)起動後: publisherは**1個だけ**のはず
   - ノード名が`_NODE_NAME_UNKNOWN_`と表示される場合(Issue #7)は
     `ros2 node info --no-daemon /robot1/controller_server`等、`--no-daemon`付きの
     ノード側からの確認で代替できる(daemonのグラフキャッシュがHumble⇔Jazzy混在で
     壊れているだけで、通信自体は正常)

3. **計測でみるトピックの整理**:

   | トピック | 中身 | 用途 |
   |---------|------|------|
   | `/cmd_vel_raw` | 自作`controller_server`(DWB)の生出力 | 追従指令そのものの記録 |
   | `/robot1/cmd_vel` | `cmd_vel_safety`経由の最終駆動指令 | ロボットに実際に届く指令の記録 |
   | `/go2_localization/tf` | 自作EKF/AMCLの`map→odom→base_link` | 到達判定・ゴール姿勢誤差の算出 |
   | `/goal_pose` | 計測試行ごとのゴール | 試行の開始トリガ・正解値 |

   自作パイプラインの「実出力」は`/cmd_vel_raw`(launch内で`cmd_vel`をremap済み)であり、
   `/robot1/cmd_vel`はあくまで`cmd_vel_safety_node`の手動remapを経た後の最終段である点に注意。

## 動作確認結果(2026-07-14、Issue #21)

フェーズA(upstream本家の`/robot1/tf`)・フェーズB(自作EKF/AMCLの`/go2_localization/tf`、
`go2_localization`のTF配信を有効化した後)の両方で、上記手順どおりにGazebo上のGo2が
実際にゴールまで到達することを確認した。

詰まったところ: `straight_line_planner_node`を`ros2 run`で単体起動する際に`use_sim_time`の
指定を忘れ、壁時計でPathをスタンプしていたため`controller_server`側で
`Transform data too old when converting from map to odom`が出続けて`Failed to make progress`に
なった(sim連携ノードは`use_sim_time:=true`の明示が必須、という基本の再確認)。

未実施:

- MPPIとの比較(Issue #22)
- ローカルコストマップでの障害物回避(Issue #23、追M3)
- ゴール姿勢誤差の定量評価(計画書M2の完了条件の精密な検証。GATE1の到達成功率・
  部材正対精度のベースライン化もここに含む)
