# 3D点群スキャンマッチング(NDT、scanmatcher)起動手順(Issue #13)

`docs/手順/3D-LIOオドメトリ(RKO-LIO)起動手順.md`でのRKO-LIO検証の過程で、ユーザーが
Unitree純正アプリの3D LiDARマッピング機能(同じセンサーで綺麗な点群地図を生成)を提示し、
「なぜ自分たちの結果はうまくいかないのか」と指摘したことを機に、2026-11-12に発見・検証した
別アプローチの記録。**現時点ではこちらが本命候補。**

## 1. 経緯(なぜRKO-LIOから乗り換えたか)

RKO-LIOはバッファ暴走(再配置探索の重さが原因)を特定・解決したが、全フレームを処理すると
実際の精度は最終誤差12.58m・5m超ズレ18%(`walk_1111`)と、実用レベルには届かなかった
(詳細は`3D-LIOオドメトリ(RKO-LIO)起動手順.md`参照)。

ユーザーの指摘を受けてRKO-LIOのソースを確認したところ、**RKO-LIOにはロボット自身の
脚オドメトリ(関節エンコーダ+接地判定による、歩行の衝撃・振動を踏まえた本体純正の
自己位置推定)を取り込む仕組みが一切なく、IMU生データのみで姿勢予測している**ことが
判明。Unitree純正アプリはおそらく脚オドメトリを主たる姿勢予測源とし、LiDARは地図に対する
補正にのみ使っていると推測し、同じリポジトリ内の別フロントエンド`scanmatcher`
(RKO-LIOより古い、NDT/GICPベースの3D点群マッチングパッケージ)に、外部オドメトリを
事前情報として使う`use_odom`機能が実装されていることを発見、検証した。

## 2. 検証結果(2026-11-12訂正済み)

**【重要な訂正】** 当初「最終誤差0.01〜0.02m」と報告していたが、これは**今日生成した
全ての軌跡ログファイルの末尾に混入していた偽のデータ点**(座標(0,0,0)、他の点と比べて
異常なタイムスタンプ、`scanmatcher_node`終了時の挙動由来と推測)による誤りだった。
偽の点を除外して再計算した正しい数字は以下の通り:

**`scanmatcher`(NDT)+ `use_odom: false`(後述の理由で脚オドメトリ事前情報は無効化)が、
3回の軌跡のみのテストでは5m超ズレ0%を再現した(最終誤差は1.3〜1.6m程度)**:

| bag | 点数 | 最大ズレ | 最終誤差(訂正後) | 5m超ズレ |
|---|---|---|---|---|
| walk_1111 | 4056 | 3.75m | **1.34m** | **0%(0件)** |
| walk_1111_marker(マーカー設置、軌跡のみのテスト) | 3841 | 3.44m | **1.52m** | **0%(0件)** |
| walk_1111_sept2(9月の場所) | 1286 | 2.74m | **1.58m** | **0%(0件)** |

参考: RKO-LIO(offline_node、全フレーム処理)の`walk_1111`結果は最終誤差12.58m・
5m超ズレ18%であり、上記は全指標で上回る。

**再現性の脆さを発見・原因特定・解決済み(§6参照)**: 当初、全く同じ設定・同じbagで
地図生成用に`walk_1111_marker`を再実行したところ、最終誤差8.09m・5m超ズレ56.4%という
大幅に悪化した結果になったことがあった。原因は重力方向推定のドリフトと判明し、
`use_imu: true`に変更することで解消した(現在の標準設定は`use_imu: true`)。

**既知の制約(重要)**:
- **脚オドメトリ事前情報(`use_odom: true`)は使わない方がよい。** 当初はTFタイミング
  競合(`SpinningTransformListener`のポーリング間隔が50msと粗く、脚オドメトリTFが
  LiDAR処理に追いつかない)で事前情報がほとんど機能していなかったため、ポーリング間隔を
  2msに短縮するソースパッチを適用・再ビルドした。しかし精度は**悪化**した(5m超ズレが
  2.8%→16.4%)。脚オドメトリ事前情報の実装自体(座標系・符号・単位等)に不具合がある
  疑いが強く、原因は未特定のまま。**現時点では`use_odom: false`を推奨する**
- 上記のソースパッチ(`scanmatcher/include/scanmatcher/spinning_transform_listener.hpp`
  のポーリング間隔50ms→2ms)は`~/lidarslam_ws`(devコンテナ内、Go2_deployのgit
  リポジトリ外)にのみ適用済みで、**どこにもコミットされていない**。`use_odom: false`
  運用では実害がないため当面放置してよいが、外部クローンのローカルパッチとして残すか
  フォークして追跡するかは未決定
- ループクロージャ・最終的な地図品質(map.pcd等)は未検証(今回はオドメトリ精度のみ確認)
- 実機ライブでの動作確認は一切ない(今回は全てオフラインのbag再生)
- `scanmatcher`パッケージ一式は`ros2_ws`とは別の独立したcolconワークスペース
  (`~/lidarslam_ws/`、`lidar_slam_ros2`の一部)にある。`real_up.sh`には未組込み

## 3. ビルド(初回のみ)

`RKO-LIO起動手順.md`§3と同じ`~/lidarslam_ws/`に含まれる(`lidar_slam_ros2`の一部)ため、
追加のクローン・ビルドは不要。TF競合対策パッチ(任意、§2参照)を適用する場合のみ:

```bash
# ~/lidarslam_ws/src/lidar_slam_ros2/scanmatcher/include/scanmatcher/
#   spinning_transform_listener.hpp の spin_once(std::chrono::milliseconds(50)) を
#   milliseconds(2) に変更後:
cd ~/lidarslam_ws
colcon build --symlink-install --packages-select scanmatcher
```

## 4. パラメータファイル

このGo2向けの設定は本リポジトリにコミット済み:
`ros2_ws/src/go2_localization/config/scanmatcher_go2.yaml`

要点(詳細な理由はファイル内コメント参照):
- `registration_method: "NDT"` — RKO-LIOのICP系より、この疎な顎LiDARデータに対して
  頑健だったアルゴリズム
- `ndt_resolution: 0.3` / `vg_size_for_input: 0.1` / `vg_size_for_map: 0.05` /
  `trans_for_mapupdate: 0.2` / `scan_max_range: 15.0` — 部屋スケールに合わせて
  既定値(大規模・屋外向け)から縮小
- `use_odom: false` — 脚オドメトリ事前情報は実装不具合の疑いがあり無効化(§2参照)
- `use_imu: true` — IMUによるデスキュー・重力/姿勢補正を有効化。重力ドリフト対策として
  必須(§6参照)

## 5. 起動(オフライン再生での検証手順)

録画済みbag(例: `ros2_ws/bags/walk_1111`)での検証は、`scanmatcher_node`単体を起動し、
外部で`ros2 bag play`する構成で確認済み:

```bash
source /opt/ros/humble/setup.bash
source ~/lidarslam_ws/install/setup.bash

ros2 run scanmatcher scanmatcher_node --ros-args \
  -r input_cloud:=/utlidar/cloud_base_restamped \
  -r imu:=/go2_state_bridge/imu \
  --params-file /tmp/scanmatcher_go2.yaml   # ros2_ws/.../scanmatcher_go2.yamlをコピーしたもの

# 別ターミナルで録画データ再生
ros2 bag play ~/ros2_ws/bags/walk_1111 --clock 100 \
  --topics /utlidar/cloud_base_restamped /go2_state_bridge/imu /tf_static
```

推定姿勢は`current_pose`トピック(`geometry_msgs/msg/PoseStamped`)に出力される。
`use_odom: true`を試す場合は、別途ロボットの脚オドメトリ(`/utlidar/robot_odom_restamped`)
をTF(`odom→base_link`)に変換する中継ノードが必要(簡易スクリプトで対応可能)。

## 6. 再現性の脆さ・重力ドリフトの原因特定と解決(2026-11-12)

`walk_1111_marker`の地図生成用の再実行で、最終誤差8.09m・5m超ズレ56.4%という
大幅に悪化した結果が出たことがあった。軌跡を詳しく見ると、**Z座標・開始点からの距離が
滑らかに単調増加し続ける**(ランダムなジャンプではない)挙動だった。

`use_imu: false`は、デスキューだけでなく**IMUによる重力・姿勢(ロール・ピッチ)補正も
同時に無効化していた**(ソース確認済み)。これにより、外部の重力基準が一切ない状態で
3D NDTマッチングのみから姿勢を推定しており、徐々に傾きが蓄積して破綻した可能性が
高いと推測。

`use_imu: true`に変更し、2つの異なるbag(`walk_1111`は2回、`marker`・`sept2`は1回ずつ、
計4回)で再検証した結果:

| テスト | 最終誤差 | 最大ズレ | 5m超ズレ | Z座標の挙動 |
|---|---|---|---|---|
| marker | 1.35m | 4.91m | 0% | ノイズ状(単調増加なし) |
| walk_1111(1回目) | 2.43m | 4.13m | 0% | - |
| walk_1111(2回目) | 1.88m | 4.50m | 0% | ノイズ状(単調増加なし) |
| sept2 | 2.03m | 2.72m | 0% | ノイズ状(単調増加なし) |

**4回全てで致命的なドリフトは再発しなかった。** 標準設定を`use_imu: true`に変更済み
(`scanmatcher_go2.yaml`)。

## 7. 地図品質の検証(2026-11-12)

`/map`トピック(PointCloud2、蓄積地図)を記録・可視化した結果:

- **統計的外れ値除去フィルタ**(k近傍法、k=10、中央値+3標準偏差を閾値、
  `scipy.spatial.cKDTree`で実装)が有効。除去率はわずか1〜2%だが、10m以上離れた
  極端な孤立点が除去され、地図の広がりが現実的な部屋スケールまで圧縮される
- **高さ帯で分けて見れば、鮮明な直線の壁が確認できる**(`walk_1111`の良好な軌跡では
  床上0.7〜1.5m付近の帯で壁・コーナー構造が明確)。3D点群全体を一括りに見ると
  もやっとした塊に見えるのは、低い高さ帯の密集(ロボットの滞在・近距離観測等)が
  壁の構造を覆い隠しているため
- **GICP(`registration_method: "GICP"`)・範囲適応フィルタは不採用**: GICPはこの
  データでは収束不能(却下16,058件)・計算負荷過大(CPU 1148%)。範囲適応フィルタは
  撮影時の実距離データが残っていないため有効な近似ができず、固定しきい値フィルタと
  有意差なし

## 9. `real_up.sh`への組込み(2026-11-12)

`docker/driver/real_up.sh`に`--scanmatcher`オプションを追加した。既存の2D経路
(height_slice_viz→slam_toolbox)には一切手を加えず、並走する独立経路として追加:

```bash
./docker/driver/real_up.sh --scanmatcher          # 観測系+scanmatcher
./docker/driver/real_up.sh --motion --scanmatcher # 走行系も含めて全部
```

- 入力はdriver側で既に中継済みの`/utlidar/cloud_base_restamped`・`/go2_state_bridge/imu`
  (2D経路と共用、追加の中継ノードは不要)
- `global_frame_id`は標準設定の`map`から`sm_map`に実行時オーバーライドしている。
  slam_toolboxが管理する既存の`map`フレーム(`map->odom`)とTFの親が競合するのを避けるため
  (scanmatcherは`use_odom:false`のため`map->base_link`を直接発行、EKFの`odom->base_link`
  とは衝突しない)
- 推定姿勢: `/current_pose`、地図: `/map`(PointCloud2)。RVizでは`Fixed Frame: sm_map`
- **前提: `ros2_ws`のcolcon buildが必要**(`scanmatcher_go2.yaml`が
  `install/go2_localization/share/.../config/`に反映されるまで)。`~/lidarslam_ws`側は
  ビルド済み前提(§3参照)
- **未実施(ユーザーが実機で検証すること)**: 本結合は机上でのスクリプト編集のみで、
  実機での起動確認は行っていない

## 10. 今後(未着手)

- 実機での`--scanmatcher`起動確認(上記、最優先)
- ループクロージャの検証(今回はスキャンマッチングのみ、バックエンドのループクロージャ
  機能は未検証)
- 脚オドメトリ事前情報の不具合原因調査(優先度低、無効化で十分良い結果が出ているため)
- 高さ帯スライス→2D occupancy grid化を正式な変換パイプラインとして整備
- `scanmatcher`採用を正式決定する場合、RKO-LIO関連ドキュメントとの整理(本命はこちらに
  切替、RKO-LIO側は「検証したが不採用」という位置づけに変更)
