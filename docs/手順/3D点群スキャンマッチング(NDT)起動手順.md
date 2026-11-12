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

## 2. 検証結果(確定)

**`scanmatcher`(NDT)+ `use_odom: false`(後述の理由で脚オドメトリ事前情報は無効化)が、
3つの異なる録画データ全てで安定して再現する、現時点で最良の設定**:

| bag | 点数 | 最大ズレ | 最終誤差 | 5m超ズレ |
|---|---|---|---|---|
| walk_1111 | 4056 | 3.75m | 0.01m | **0%(0件)** |
| walk_1111_marker(マーカー設置) | 3841 | 3.44m | 0.02m | **0%(0件)** |
| walk_1111_sept2(9月の場所) | 1286 | 2.74m | 0.01m | **0%(0件)** |

参考: RKO-LIO(offline_node、全フレーム処理)の`walk_1111`結果は最終誤差12.58m・
5m超ズレ18%であり、全指標で明確に上回る。

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

## 6. 今後(未着手)

- ループクロージャ・最終的な地図品質(map.pcd)の検証(今回はオドメトリ精度のみ)
- 脚オドメトリ事前情報の不具合原因調査(優先度低、無効化で十分良い結果が出ているため)
- `real_up.sh`への組込み(新しい追加起動オプションとして、既存の2D経路は維持する方針)
- 実機ライブでの検証(今回はオフラインbag再生のみ)
- `scanmatcher`採用を正式決定する場合、RKO-LIO関連ドキュメントとの整理(本命はこちらに
  切替、RKO-LIO側は「検証したが不採用」という位置づけに変更)
