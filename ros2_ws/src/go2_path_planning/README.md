# go2_path_planning — 経路生成（Nav2 planner_server）

経路生成サブシステム（生M2: コストマップ + グリッド探索、Issue #16/#17）の
bringup パッケージ。自作ノードは `plan_requester`（小さな橋渡し）1つだけで、
経路計画の本体は既製の Nav2 `planner_server`（NavFn=**ダイクストラ法**）を
設定ファイル + 起動ファイルで駆動する構成。

---

## 使い方

### ビルド

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select go2_path_planning
source install/setup.bash
```

### 起動（フェーズB構成: 自作TF）

devコンテナ + simコンテナの両方を起動した状態で、別ターミナルでそれぞれ立ち上げる。
`straight_line_planner`（生M1）の代わりに下記の 2 つ（`planner.launch.py` と
`plan_requester`）を使うだけで、それ以外は `go2_path_following/README.md` の
「使い方（フェーズB）」と同じ。

```bash
# 自己位置推定（map_server を含む。static_layer の地図配信元）
ros2 launch go2_localization localization.launch.py

# planner_server + lifecycle_manager（このパッケージ）
ros2 launch go2_path_planning planner.launch.py

# goal_pose → ComputePathToPose → plan の橋渡しノード（このパッケージ）
ros2 run go2_path_planning plan_requester

# 経路追従チェーン
ros2 launch go2_path_following controller.launch.py
ros2 run go2_path_following plan_follower
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args -r cmd_vel:=/robot1/cmd_vel
```

### ゴール投入

`go2_path_following` のスクリプトをそのまま使う（引数は `x y yaw[deg]`）。

```bash
~/ros2_ws/src/go2_path_following/scripts/send_goal.sh -1.71 4.70 90
```

### プランナIDの指定（任意）

`plan_requester` は `planner_id` パラメータで planner_server 側のプラグイン名を
選ぶ。既定は `GridBased`（`planner_server.yaml` の `planner_plugins` と一致）。

```bash
ros2 run go2_path_planning plan_requester --ros-args -p planner_id:=GridBased
```

### 動作確認

`plan_requester` のログに `published path: N points, planning_time=...s` が出れば
計画は成功している。生成された経路そのものは `plan` トピックで確認する。

```bash
# plan_requester のログに ready と published path が出るか
# 生成された Path を直接見る（QoS を合わせないと届かない点に注意）
ros2 topic echo /plan --qos-durability transient_local --once
```

---

## 概要

### サブシステムの位置づけ（計画書 M2）

本パッケージは経路生成の「生M2: コストマップ + グリッド探索」に対応する
（`docs/計画/経路生成.md`）。生M1 の `straight_line_planner`（直線経路）を、
コストマップを使った既知障害物回避のグリッド探索プランナに差し替えるステップ。

Nav2 の `planner_server` は本来 `bt_navigator` から `ComputePathToPose`
アクションで駆動される。しかし本構成では `plan_follower`（経路追従側）と同様に
`bt_navigator` を導入せず、`plan_requester` が `goal_pose` を購読して自分で
アクションを呼び、結果の `Path` を `plan` トピックに流す最小構成にとどめる。

### データフロー

```
goal_pose (PoseStamped)
    │  ［plan_requester が購読］
    ▼
plan_requester ──ComputePathToPose アクション──▶ planner_server (NavFn)
    │                                                    │
    │  ◀────────── 結果 Path ─────────────────────────────┘
    ▼
plan (Path, QoS=TRANSIENT_LOCAL)
    │  ［下流の plan_follower が購読］
    ▼
plan_follower → controller_server(DWB) → cmd_vel_safety → /robot1/cmd_vel
```

`straight_line_planner`（生M1）とトピックI/F（`goal_pose` / `plan`、QoS 含む）を
**完全に揃えてある**ため、下流（`plan_follower` → `controller_server` →
`cmd_vel_safety`）はどちらのプランナが動いているか気づかない。差し替えは
「どちらを起動するか」だけで済む。

### 入出力（`plan_requester` ノード）

ノード名: `plan_requester`

| 種別 | 名前 | 型 | 備考 |
|------|------|-----|------|
| Subscribe | `goal_pose` | `geometry_msgs/PoseStamped` | 目標姿勢。QoS depth=10（既定） |
| Publish | `plan` | `nav_msgs/Path` | 生成経路。QoS: RELIABLE / TRANSIENT_LOCAL / depth=1（latched 相当） |
| Action Client | `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | planner_server が提供 |
| Parameter | `planner_id` | string | 既定 `GridBased`。planner_server のプラグイン名 |

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
| `plugins` | `["static_layer", "inflation_layer"]` | 生M2 は 2 層構成（`docs/計画/経路生成.md` §3.3） |
| `always_send_full_costmap` | `true` | 差分ではなく全コストマップを配信 |

**static_layer**（既知地図）
- `plugin`: `nav2_costmap_2d::StaticLayer`
- `map_topic`: `/go2_localization/map` — 自作 `map_server`（`go2_localization` が起動）の
  地図を購読。upstream（`/robot1/...`）とは分離した専用トピック
- `map_subscribe_transient_local`: `true`

**inflation_layer**（膨張）
- `plugin`: `nav2_costmap_2d::InflationLayer`
- `cost_scaling_factor`: `4.0` — 障害物からの距離に対するコスト減衰率
- `inflation_radius`: `0.45` — 膨張半径[m]

> obstacle layer（顎LiDARのライブスキャン）は生M3（Issue #18）で追加する予定。
> 現状の 2 層では動的障害物・未知障害物は避けられない。

#### lifecycle_manager_planning

- `autostart: true` — 起動時に自動でライフサイクルをアクティブ化
- `node_names: ["planner_server"]` — 管理対象は planner_server のみ

### `plan_requester.py` の内部ロジック

1. `goal_pose`（`PoseStamped`）を購読（QoS depth=10）。
2. ゴール受信時、`compute_path_to_pose` アクションサーバの起動を最大 1.0 秒待機。
   未起動なら 5 秒スロットルの警告を出して return（サーバが立つまで再送で対応）。
3. `ComputePathToPose.Goal` を構築して送信:
   - `goal`: 受信した `PoseStamped` をそのまま設定
   - `planner_id`: パラメータ `planner_id`（既定 `GridBased`）
   - `use_start = False`: 始点を指定せず、現在位置（TF）を始点に使わせる
4. ゴールが reject されたら警告して終了（`ComputePathToPose goal rejected`）。
5. 結果の `path.poses` が空なら「計画結果が空（ゴールが障害物内/コストマップ外、
   または TF 未取得の可能性）」と警告して終了。
6. 経路が得られたら `plan` トピックへ publish し、点数と `planning_time` をログ出力。

`plan` トピックの QoS は RELIABLE / TRANSIENT_LOCAL / depth=1。TRANSIENT_LOCAL に
より、後から接続した subscriber にも最新の Path が届く（latched 相当）。これは
`straight_line_planner` と揃えるための設計。

### 設計判断の理由

- **`bt_navigator` を使わない**: リカバリ・BT ツリーの複雑さを避け、生M1〜M2 の
  範囲では「goal → 単一の Path → 追従」の最小フローに集中するため。
- **トピックI/F を生M1 と完全一致**: プランナの中身（直線 vs グリッド探索）を
  下流に一切影響させずに差し替え可能にするため。
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

---

## 詰まりどころ（既知）

- **`/goal_pose` の配送が間欠的に遅延する**ことがある（Humble⇔Jazzy 混在環境、#7 の
  周辺症状と思われる）。`ros2 topic pub --once` は論外（マッチング前 publish で
  消える）、`-w 1 -t 2` でも届かないことがあり、後続の送信でまとめて届く挙動を
  観測した。`send_goal.sh` は複数回送信にしてあるが、無反応なら
  `ros2 topic echo /plan --qos-durability transient_local --once` で Path の有無を
  見てもう一度送るのが早い（`plan_requester` のログに `published path` が出れば
  計画は済んでいる）。
- planner_server が「計画結果が空」を返す場合: ゴールが障害物内（インフレーション
  込み）かコストマップ外。ゴール位置を少しずらす。

---

## 未実施

- MPPI/Smac 系との比較（まずは NavFn ダイクストラを既定とする）。
- obstacle layer 追加（生M3、Issue #18）・再計画のトリガ設計（経路無効化時）。
- 経路品質の定量評価（障害物配置を変えた成功率・経路長・計画時間）。生M2 完了条件
  の「シミュレーション → 実機」の実機側も未実施。
