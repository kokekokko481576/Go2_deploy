# go2_path_planning — 経路生成（Nav2 planner_server）

経路生成サブシステム（生M2: コストマップ + グリッド探索、Issue #16/#17。
生M3: obstacle_layer追加、Issue #18）の bringup パッケージ。経路計画の本体は
既製の Nav2 `planner_server`（NavFn=**ダイクストラ法**）を設定ファイル + 起動
ファイルで駆動する構成。かつては自作の`plan_requester`ノードが`goal_pose`を
購読して`ComputePathToPose`アクションを叩いていたが、Issue #50でBT化した後は
`nav2_bt_navigator`（`go2_path_following`側）が直接このアクションを呼ぶため、
`plan_requester`は削除済み。現在このパッケージに自作ノードは無く、設定ファイル+
起動ファイルのみの純粋なbringupパッケージになっている。

---

## 使い方

### ビルド

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_path_planning
source install/setup.bash
```

### 起動（BT導入後の既定）

Issue #50でBT化された現在、このパッケージを単体で手動起動する場面は通常無い。
フルスタックは統合launchでまとめて起動する（詳細は
`go2_path_following/README.md`「使い方(BT導入後の既定)」参照）:

```bash
ros2 launch ~/ros2_ws/launch/demo.launch.py
```

`planner.launch.py`（このパッケージ）が起動するのは`planner_server`+
`lifecycle_manager_planning`のみで、`goal_pose`から`ComputePathToPose`を
駆動する役目は`nav2_bt_navigator`（BT XMLは`go2_bt_plugins`）が直接担う。
`planner_server`単体をデバッグしたい場合のみ、このパッケージだけを個別に
起動できる:

```bash
# 自己位置推定（map_server を含む。static_layer の地図配信元）
ros2 launch go2_localization localization.launch.py

# planner_server + lifecycle_manager（このパッケージ）
ros2 launch go2_path_planning planner.launch.py
```

### ゴール投入

`go2_path_following` のスクリプトをそのまま使う（引数は `x y yaw[deg]`）。

```bash
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh -1.71 4.70 90
```

### プランナIDの指定

以前は自作`plan_requester`の`planner_id`パラメータで切り替えられたが、BT化後は
BT XML（`go2_bt_plugins/behavior_trees/navigate_with_collision_recovery.xml`）の
`<ComputePathToPose ... planner_id="GridBased"/>`に固定されている。既定の
`GridBased`（`planner_server.yaml` の `planner_plugins` と一致）から切り替えたい
場合は、BT XML側のこの属性を書き換える必要がある。

### 動作確認

`plan_requester`は削除済みのため、計画成功の確認は`bt_navigator`のログ
（`ComputePathToPose`の成否）と`plan`トピックそのもので行う。

```bash
# 生成された Path を直接見る（QoS を合わせないと届かない点に注意）
ros2 topic echo /plan --qos-durability transient_local --once
```

---

## 概要

### サブシステムの位置づけ（計画書 M2）

本パッケージは経路生成の「生M2: コストマップ + グリッド探索」に対応する
（`docs/計画/経路生成.md`）。生M1 の `straight_line_planner`（直線経路）を、
コストマップを使った既知障害物回避のグリッド探索プランナに差し替えるステップ。

Nav2 の `planner_server` は `bt_navigator` から `ComputePathToPose` アクションで
駆動される。当初（生M2導入時）は `bt_navigator` を導入せず、自作の `plan_requester`
が `goal_pose` を購読して自分でアクションを呼ぶ最小構成にとどめていたが、
Issue #50で `bt_navigator` を導入した後は `plan_requester` を廃し、`bt_navigator`
が直接このアクションを呼ぶ構成に一本化した（下記「データフロー」参照）。

### データフロー（BT導入後）

```
goal_pose (PoseStamped)
    │  ［goal_pose_bridge が購読、go2_path_following］
    ▼
goal_pose_bridge ──NavigateToPose アクション──▶ bt_navigator
                                                     │  ComputePathToPose アクション
                                                     ▼
                                              planner_server (NavFn)
                                                     │  結果 Path
                                                     ▼
                                    plan (Path, QoS=TRANSIENT_LOCAL、可視化用)
                                                     │
                                          FollowPath アクション経由でそのまま
                                                     ▼
                                    controller_server(DWB) → cmd_vel_safety → /robot1/cmd_vel
```

`straight_line_planner`（生M1）とはトピックI/F（`goal_pose` / `plan`、QoS 含む）
を揃えているが、BT導入後の既定フローは`goal_pose_bridge`→`bt_navigator`経由の
アクション呼び出しに一本化されており、`straight_line_planner`は単体起動での
デバッグ用途にのみ残っている（詳細は`go2_path_following/README.md`
「設計の変遷」参照）。

### 入出力（`planner_server`）

本パッケージに自作ノードは無く、`planner_server`（`nav2_planner`）を素のまま
起動するだけ。`ComputePathToPose`アクションは`bt_navigator`（`go2_path_following`
側）がBT XMLから直接呼ぶ。

| 種別 | 名前 | 型 | 備考 |
|------|------|-----|------|
| Action Server | `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | `bt_navigator`から`planner_id="GridBased"`固定で呼ばれる |
| Publish | `plan` | `nav_msgs/Path` | `planner_server`本体が計画結果を可視化用に配信 |

### 起動されるノード（`planner.launch.py`）

| ノード | パッケージ / 実行ファイル | 役割 |
|--------|---------------------------|------|
| `planner_server` | `nav2_planner` / `planner_server` | 経路計画本体（NavFn + global_costmap） |
| `lifecycle_manager_planning` | `nav2_lifecycle_manager` / `lifecycle_manager` | `planner_server` を autostart でアクティブ化 |

両ノードとも TF リマップを行い、フェーズB配線（自作 AMCL/EKF の専用TF）を参照する:
`/tf` → `/go2_localization/tf`、`/tf_static` → `/robot1/tf_static`。

---

## 詳細

### `planner_server.yaml` の主要パラメータ

#### planner_server

| パラメータ | 値 | 意味 |
|-----------|-----|------|
| `use_sim_time` | `true` | シミュレーション時刻を使用 |
| `expected_planner_frequency` | `1.0` | 計画要求が来る想定頻度の**警告しきい値のみ**。実際の計画はアクション要求ごと |
| `planner_plugins` | `["GridBased"]` | 使用プランナのプラグイン名リスト |

#### GridBased プラグイン（NavFn）

| パラメータ | 値 | 意味 |
|-----------|-----|------|
| `plugin` | `nav2_navfn_planner/NavfnPlanner` | NavFn プランナ |
| `tolerance` | `0.5` | ゴール到達不能時に許容する最終地点までの距離[m] |
| `use_astar` | `false` | **ダイクストラ法**（Issue #16 の本命）。`true` で A* に切替 |
| `allow_unknown` | `true` | 未知セル（track_unknown_space）を通過可能とする |

#### global_costmap

| パラメータ | 値 | 意味 |
|-----------|-----|------|
| `update_frequency` / `publish_frequency` | `1.0` / `1.0` | コストマップ更新・配信頻度[Hz] |
| `global_frame` / `robot_base_frame` | `map` / `base_link` | 座標系 |
| `rolling_window` | `false` | 地図全域を対象（ローリング窓を使わない） |
| `track_unknown_space` | `true` | 未知空間を区別して扱う |
| `resolution` | `0.05` | セル解像度[m] |
| `footprint` | `[[0.35,0.18],[0.35,-0.18],[-0.38,-0.18],[-0.38,0.18]]` | ロボット外形。`go2_path_following` の local_costmap と同じ共有定義（C5） |
| `plugins` | `["static_layer", "obstacle_layer", "inflation_layer"]` | 生M2(static+inflation)に生M3(obstacle_layer)を追加した3層構成（`docs/計画/経路生成.md` §3.3） |
| `always_send_full_costmap` | `true` | 差分ではなく全コストマップを配信 |

**static_layer**（既知地図）
- `plugin`: `nav2_costmap_2d::StaticLayer`
- `map_topic`: `/go2_localization/map` — 自作 `map_server`（`go2_localization` が起動）の
  地図を購読。upstream（`/robot1/...`）とは分離した専用トピック
- `map_subscribe_transient_local`: `true`

**obstacle_layer**（顎LiDARのライブスキャン、生M3・Issue #18）
- `plugin`: `nav2_costmap_2d::ObstacleLayer`
- `observation_sources`: `chin_lidar_scan` — `go2_localization`が自M2用に既に作っている
  顎LiDAR→2Dスキャン（`/go2_localization/chin_lidar_scan`、`pointcloud_to_laserscan`の出力、
  `base_link`基準に変換済み・脚の近距離ノイズはrange_min=0.4で除去済み）をそのまま流用する。
  新たな点群処理は追加していない
- `data_type: "LaserScan"` / `marking: true` / `clearing: true` — マーク（障害物を立てる）・
  レイトレースクリア（光線が通った空間の障害物マークを消す）を両方行う
- `obstacle_max_range: 10.0` / `raytrace_max_range: 10.5` — この距離まではマーク・クリア対象
- `inf_is_valid: true` — 上流の`pointcloud_to_laserscan`が`use_inf: true`(範囲内に何もなければ
  `inf`を返す)ため、`inf`を「そこまで障害物なし」の有効なクリア観測として扱う設定が必要
  （`false`だと`inf`の観測が丸ごと捨てられ、見えているはずの自由空間がクリアされない）

**inflation_layer**（膨張）
- `plugin`: `nav2_costmap_2d::InflationLayer`
- `cost_scaling_factor`: `4.0` — 障害物からの距離に対するコスト減衰率
- `inflation_radius`: `0.45` — 膨張半径[m]

> 地図にある既知障害物はstatic_layer、地図に無い障害物（顎LiDARが今見ているもの）は
> obstacle_layerがそれぞれ担当し、両方の上にinflation_layerが安全マージンを掛ける。
> 「経路無効化時のみ・1Hz上限」という当初の再計画トリガ設計はまだ実装していないが、
> Issue #50でBT化した際にstockのBTテンプレートを土台にしたため、結果的に
> `RateController hz="1.0"`によって**常時1Hzで再計画**される動作になっている
> （詳細・経緯は下記「未実施」参照）。

#### lifecycle_manager_planning

- `autostart: true` — 起動時に自動でライフサイクルをアクティブ化
- `node_names: ["planner_server"]` — 管理対象は planner_server のみ

### （Issue #50で削除済み）`plan_requester` の内部ロジック

かつては自作`plan_requester`が「`goal_pose`購読→`ComputePathToPose`アクション
呼び出し→`plan`配信」を行っていたが、このノード自体をIssue #50で削除した。
現在は`bt_navigator`がBT XML側の`<ComputePathToPose goal="{goal}" path="{path}"
planner_id="GridBased"/>`ノードとして同アクションを直接呼ぶ（詳細は
`go2_bt_plugins/README.md`参照）。

### 設計判断の理由

- **（Issue #50で撤回）当初は`bt_navigator`を使わない方針だった**: リカバリ・BT
  ツリーの複雑さを避け、生M1〜M2の範囲では「goal → 単一の Path → 追従」の最小
  フローに集中する狙いだったが、接触検知→リカバリの判断ロジックが複雑化したため
  Issue #50で`nav2_bt_navigator`導入に切り替えた（経緯は`go2_path_following/README.md`
  「設計の変遷」参照）。
- **トピックI/F を生M1 と完全一致**: プランナの中身（直線 vs グリッド探索）を
  下流に一切影響させずに差し替え可能にするため（`plan`トピックは可視化用として
  現在も維持）。
- **地図トピックを upstream と分離**（`/go2_localization/map`）: 自作の自己位置推定
  スタックが配信する地図を使い、シミュレータ既定の `/robot1/...` と混線させない。
- **NavFn ダイクストラを既定**: まずは素直な全探索系を基準（ベースライン）として
  確立する。A*（`use_astar: true`）や Smac/MPPI 系との比較は後段。

---

## 動作確認結果（2026-07-17、Issue #16/#17）

cafe ワールド（Gazebo）・フェーズB（自作TF）で確認:

- **#16（最小起動）**: planner_server + global_costmap（static+inflation）が
  アクティブ化し、ゴール (2.5, 3.0) に対し 156 点の Path を生成（計画時間 0.001s）。
  障害物のない方向では経路長 3.92m vs 直線 3.91m でほぼ直線 = 妥当。
- **#17（既知障害物回避）**: 直線が地図上の障害物（-1.2, 3.4 付近）を横切るゴール
  (-1.71, 4.70) に対し、経路長 5.32m vs 直線 5.00m（+0.32m）の**迂回経路**を生成。
  そのままフルチェーン（plan_follower → DWB → cmd_vel_safety）でロボットが実際に
  迂回歩行し `Reached the goal!`（最終誤差約 0.18m、トレランス 0.25m 内）を確認。

## 動作確認結果（2026-08-03、Issue #18: obstacle_layer追加）

cafe ワールド（Gazebo）で、**地図に無い実在の障害物**（cafeワールドのテーブル状の物体。
map.pgmの画素値では自由空間扱い。`OccupancyGrid`の値体系（自由空間=0/占有=100/未知=-1）とは
別の話なので注意）を使って確認:

- **マーキング確認**: 顎LiDARの実測（`/go2_localization/chin_lidar_scan`の非`inf`ヒットを
  角度・距離から世界座標に逆算した点、例: (0.60, -1.60)）と同じ位置の`/global_costmap/costmap`
  セルが`cost=100`（リーサル障害物）・周辺セルが`cost=99`（膨張域）になっていることを確認。
  地図には存在しない障害物が、ライブスキャンだけでコストマップに立っていることが分かる
- **迂回確認**: 上記障害物の近くを直線が通過するゴール(0.6, -2.5)に対し、
  経路長 2.76m vs 直線 2.57m（+0.19m）の**迂回経路**を生成。障害物のy範囲
  （y≈-0.5〜-1.7付近）を大きく回り込んでから障害物を過ぎた後にゴール方向へ向かう
  軌跡になっており、obstacle_layerが実際に効いていることを確認
- 合成の箱（0.3〜0.5m角）をGazeboにspawnして正面1〜1.2mに置く方法も試したが、顎LiDARの
  下向き20°固定チルト+16ch(±15°)という粗い垂直分解能では、狭い物体だと都合よく
  ビームが当たらず`inf`のまま検出できないことがある(この環境の顎LiDAR自体の特性。
  自M2側の既知の課題でもある)。実在の大きな障害物（テーブル等）なら安定して検出できた

---

## 詰まりどころ（既知）

- **`/goal_pose` の配送が間欠的に遅延する**ことがある（Humble⇔Jazzy 混在環境、#7 の
  周辺症状と思われる）。`ros2 topic pub --once` は論外（マッチング前 publish で
  消える）、`-w 1 -t 2` でも届かないことがあり、後続の送信でまとめて届く挙動を
  観測した。`send_goal.sh` は複数回送信にしてあるが、無反応なら
  `ros2 topic echo /plan --qos-durability transient_local --once` で Path の有無を
  見てもう一度送るのが早い（`plan` トピックにPathが出ていれば計画自体は済んでいる）。
- planner_server が「計画結果が空」を返す場合: ゴールが障害物内（インフレーション
  込み）かコストマップ外。ゴール位置を少しずらす。

---

## 未実施

- MPPI/Smac 系との比較（まずは NavFn ダイクストラを既定とする）。
- **再計画のトリガ設計**（経路無効化時のみ・1Hz上限、Issue #18の残り）。Issue #50で
  BT化した際、stockの`navigate_to_pose_w_replanning_and_recovery.xml`をそのまま
  土台にしたため、結果的に`RateController hz="1.0"`により**常時1Hzで再計画**される
  ようになった（「経路無効化時のみ」に絞る、という当初の設計意図とは違う形で
  「1Hz上限」の要件だけは満たされている）。再計画を無効化時のみに絞る最適化は未実施。
- 経路品質の定量評価（障害物配置を変えた成功率・経路長・計画時間）。生M2 完了条件
  の「シミュレーション → 実機」の実機側も未実施。
