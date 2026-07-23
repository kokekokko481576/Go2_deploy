# straight_line_planner — 直線経路プランナ（Phase1 検証用）

自己位置（TF）から目標作業姿勢まで**直線補間しただけ**の `nav_msgs/Path` を出す、最小構成のプランナノード。
Nav2 の本格プランナ（`go2_path_planning`）に差し替える前段の検証・デバッグ用として使う。

---

## 使い方

### 1. ビルドと起動

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select straight_line_planner
source install/setup.bash
ros2 run straight_line_planner straight_line_planner_node
```

起動可能名（entry point）は **`straight_line_planner_node`**。
`ros2 launch` 用の launch ファイルは現状同梱していない（`ros2 run` で直接起動する）。

起動に成功すると次のようなログが出て、`goal_pose` の受信待ちに入る:

```
straight_line_planner ready: goal_pose -> plan (frame=map, base=base_link, resolution=0.1m)
```

### 2. ゴールを与える

`map`→`base_link` の TF が通っている前提で、別ターミナルからゴール姿勢を publish する:

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 6.0}, orientation: {z: 0.7071, w: 0.7071}}}" --once
```

生成された経路の確認:

```bash
ros2 topic echo /plan --once
```

`/plan` は Transient Local（後述）なので、`--once` でも最後に配信された経路を受け取れる。

### 3. TF がまだ無い環境で試す場合

実機・シミュレータが無い状態でも、静的 TF を手で流せば動作確認できる:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

（`docker/` の dev コンテナ内でこの構成の動作を確認済み。「動作確認の履歴」参照）

---

## 概要

### このパッケージがすること

- `goal_pose`（`PoseStamped`）を受信する
- TF から現在の自己位置（`global_frame`→`robot_base_frame`、既定 `map`→`base_link`）を取得する
- 自己位置（start）とゴール（goal）を**直線で結ぶ経路**を、一定間隔（`path_resolution`、既定 0.1m）でサンプリングして `nav_msgs/Path` を組み立てる
- それを `plan` トピックへ配信する

障害物回避もコストマップ評価も一切行わない。文字どおり2点を直線で結ぶだけの、船内 Go2 システムにおける経路生成の**最小実装**である（経路生成計画 Phase1 M1、`docs/計画/経路生成.md` §3.2）。

### go2_path_planning（Nav2 planner）との使い分け

同じ船内 Go2 システムには、Nav2 の `planner_server`（NavFn / ダイクストラ法＋コストマップ）を用いる本格プランナ `go2_path_planning` が別に存在する。両者の関係は以下:

| | `straight_line_planner`（本パッケージ） | `go2_path_planning` |
|--|--|--|
| 位置づけ | Phase1 M1（生M1）／軽量な検証・デバッグ用 | 生M2 以降／本番の経路生成 |
| 経路 | start→goal の**直線補間のみ** | コストマップ上のグリッド探索（障害物回避あり） |
| 障害物回避 | なし | static + inflation layer で回避（obstacle layer は将来追加） |
| 駆動方式 | `goal_pose` を直接 subscribe → `plan` を publish | `plan_requester` が `ComputePathToPose` アクション経由で planner_server を叩き `plan` へ流す |

重要なのは、**両者がトピック I/F（`goal_pose` / `plan`、型、QoS）を完全に揃えている**点である。
下流の経路追従（`go2_path_following` の `plan_follower`）から見ると、どちらのプランナが動いていても区別がつかず、差し替えは「どちらのノードを起動するか」だけで済む。
本パッケージのソースコメントでも「Nav2 の planner_server が最終的に差し替わっても購読側が気づかないよう、トピック名・型を Nav2 に合わせている」と明記されている。

したがって使い分けは:

- **`straight_line_planner`**: 障害物を無視してよい素性確認、経路追従・下流チェーン単体の動作確認、TF/座標系まわりの切り分けなど、軽量に回したいとき。
- **`go2_path_planning`**: 実際に障害物を避けた経路が必要な本番相当の検証以降。

### 入出力

ノード名: **`straight_line_planner_node`**

| 種別 | 名前 | 型 | 備考 |
|------|------|-----|------|
| 購読 | `goal_pose` | `geometry_msgs/PoseStamped` | 目標作業姿勢（部材への正対点）。Phase1 では手動 publish 可 |
| 配信 | `plan` | `nav_msgs/Path` | Nav2 `planner_server` の既定出力トピック名に合わせている |
| TF（購読） | `global_frame` → `robot_base_frame` | — | 現在の自己位置。既定は `map`→`base_link` |

`plan` の QoS は **depth=1 / RELIABLE / TRANSIENT_LOCAL**。
これも Nav2 の planner 出力に揃えたもので、後から起動した購読側でも直近の経路を取りこぼさない。

---

## 詳細

### パラメータ

| 名前 | 型 | 既定値 | 意味 |
|------|-----|--------|------|
| `global_frame` | string | `map` | 経路を表現する基準フレーム。TF ルックアップの親フレームであり、生成する `Path`・各 `PoseStamped` の `frame_id` にもなる |
| `robot_base_frame` | string | `base_link` | 自己位置とみなすロボット本体フレーム。TF ルックアップの子フレーム |
| `path_resolution` | double | `0.1` | 経路点の間隔 [m]。小さいほど点が密になる |

### 直線経路の生成ロジック

`goal_pose` を受信するたびに次の処理を行う（実装は `_on_goal` / `_interpolate`）:

1. **自己位置の取得**: `lookup_transform(global_frame, robot_base_frame, Time())`（`Time()`＝最新）で TF を引き、その並進・回転を start 姿勢とする。TF が引けない場合は警告ログを出して**経路を出さずに return**（このゴールはスキップ）。
2. **距離と進行方向**: start→goal の差分から距離 `distance = hypot(dx, dy)` と進行方向 `heading = atan2(dy, dx)` を計算する。距離が `1e-6` 未満（＝ほぼ同一点）のときは `heading` を start の現在ヨー角に維持する（ゼロ距離での向き暴れを防ぐため）。
3. **サンプリング**: 区間数を `num_segments = max(1, ceil(distance / path_resolution))` とし、`i = 0..num_segments` の各点を `t = i / num_segments` の比率で線形補間して `(x, y)` を置く。したがって点数は必ず `num_segments + 1` 個で、始点と終点を必ず含む。
4. **各点の姿勢（向き）**:
   - 中間点（`i < num_segments`）は進行方向 `heading` を向かせる（ヨー角→クォータニオン変換）。
   - **終端のみ** goal の `orientation` をそのまま使う（部材への正対姿勢を保持するため）。

生成する `Path`・各点の `frame_id` はいずれも `global_frame`、`stamp` は処理時刻（`now()`）で統一される。

### 座標系の扱いに関する注意

ゴールの `header.frame_id` が空でなく、かつ `global_frame` と異なる場合、
**フレーム変換は行わず**「そのまま座標を使う」旨の警告ログを出す（`_on_goal` 冒頭）。
つまりゴールは常に `global_frame` 上の座標として解釈される。異なるフレームでゴールを与えたい場合は呼び出し側で変換しておくこと。

### 設計判断の理由

- **なぜトピック名・型・QoS を Nav2 に揃えるか**: Phase2 で Nav2 の planner server（`go2_path_planning`）へ差し替える際、経路追従側の I/F を一切変えずに済ませるため。プランナ差し替えを「起動するノードの選択」だけに閉じ込める意図。
- **なぜ直線だけか**: Phase1 M1 では経路生成パイプライン（自己位置 → 経路 → 追従）を最短で通すことが目的で、障害物回避は後続フェーズ（`go2_path_planning` の生M2/M3）に委ねているため。
- **なぜ終端だけゴール姿勢を使うか**: 移動中は進行方向を向いていれば追従しやすく、到達時のみ部材への正対姿勢（作業姿勢）が必要になるため。

### 依存パッケージ

`rclpy` / `geometry_msgs` / `nav_msgs` / `tf2_ros`（`package.xml`）。

---

## 動作確認の履歴

### 単体確認（2026-07-12）

`docker/`（dev）コンテナ内で `tf2_ros static_transform_publisher` により `map`→`base_link` を固定配信した状態（実機・Gazebo なしのシミュレーション相当）で確認:

- `goal_pose` 受信 → 距離に応じた点数（0.1m 間隔）で `plan` を配信することを確認
- 中間点の向きが進行方向（atan2）になっていること、終端がゴールの姿勢と一致することを確認

### 経路追従との統合（2026-07-14）

`go2_path_following` の `plan_follower` が本ノードの `plan` を購読し、Gazebo 上でゴール到達まで確認済み（詳細は `go2_path_following/README.md`・Issue #21）。

### 未実施

- Gazebo・実機での部材正対精度・到達成功率の**定量**計測（§3.5 の完了条件）。上記統合確認は定性的な到達確認までで、精度の数値評価は GATE1 のベースライン化と合わせてこれから。
