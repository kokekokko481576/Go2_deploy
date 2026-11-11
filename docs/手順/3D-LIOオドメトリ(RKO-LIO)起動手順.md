# 3D LiDARオドメトリ(RKO-LIO)起動手順(Issue #13)

`docs/計画/自己位置推定.md` §1の「3D化(点群 vs 点群地図のマッチング)は精度不足が
確認されてから」の条件が、2026-11-11の実機SLAM検証で満たされたことに伴う記録。
2D AMCL/slam_toolbox自体の変更は行わず、3D LIOは別経路として追加検証した。

## 1. 背景(なぜ3D化を検証したか)

既存の2D経路(`height_slice_viz`で顎3D LiDARを高さ帯スライスし2D LaserScanへ変換→
`slam_toolbox`)で、地図が「中心の円形空白から放射状にノイズが広がる」形に一貫して
破損する問題があった。原因を追跡した結果:

- 顎LiDAR自体は健全(`/utlidar/cloud_base`は1スキャンあたり1,400〜1,700点が100%有効)
- **高さスライス処理自体が、有効な3D点群情報の約97%(1,500点→45点程度)を捨てている**
- その疎な2D スキャンを、密な360°LiDAR前提のslam_toolbox(相関スキャンマッチング)に
  入れていることが、aperture problem/degenerate geometry(公開文献: DARE-SLAM、
  McDermott & Rife/ICET等)を起こしている

ping不安定・部屋・9月時点の設定差異は全て切り離し済み(原因ではないと確認済み)。

## 2. 検証した3D LIOと結論

| 候補 | 結果 |
|---|---|
| Point-LIO(`dfloreaa/point_lio_ros2`) | 特徴の乏しい区間で発散。チューニング後も最良35.7mドリフト。**不採用** |
| **RKO-LIO**(`rsasaki0109/lidar_slam_ros2`内) | ループクロージャは機能せず(全区間で採用0件)だが、kidnap-relocalizationのみで最大ドリフト3.75mまで改善。**採用、一旦これで進める** |

**既知の制約(重要)**:
- `graph_based_slam_node`によるループクロージャは、今回の疎な顎LiDARデータでは
  **全区間で1件も採用されなかった**(却下673件、採用0件)。同じ場所に戻ってもfitness
  (マッチング適合度)自体は改善することがあるが、その場合は「補正量が0.5mを超える」
  別の安全基準で却下され続ける。閾値緩和は未検証(著者は緩和に対し「尤もらしいが
  間違った姿勢に収束する」リスクを警告している)
- `/go2_state_bridge/imu`とLiDARスキャンの間でIMUメッセージが0件になる瞬間が
  繰り返し発生(71件/約5分走行)。デスキュー精度悪化の一因である可能性があるが未調査
- RKO-LIOパッケージ一式は`ros2_ws`とは別の**独立したcolconワークスペース**
  (`~/lidarslam_ws/`、外部クローン10パッケージ)にある。`real_up.sh`には未組込み

## 3. ビルド(初回のみ)

`~/lidarslam_ws/`に`rsasaki0109/lidar_slam_ros2`をclone後:

```bash
cd ~/lidarslam_ws
./scripts/source_quickstart.sh --build-only
```

ビルドは外部クローンコードのため、auto-mode環境では許可プロンプトが出る
(`colcon build --symlink-install`を手動実行する想定)。

## 4. パラメータファイル

このGo2向けの上書き設定は本リポジトリにコミット済み:
`ros2_ws/src/go2_localization/config/rko_lio_go2.yaml`

要点(詳細な理由はファイル内コメント参照):
- `lidar_timestamps.force_relative: true` — `/utlidar/cloud_base_restamped`の
  `time`フィールド(スキャン内相対時刻)を自動判定できずクラッシュするため明示指定
- `voxel_size: 0.1` / `min_range: 0.3` / `max_range: 15.0` — 部屋スケールに縮小
  (既定値のままだと対応点がほぼ常に0になり即座に追跡を失う)
- `reset_on_registration_failure: true` / `enable_kidnap_relocalization: true` —
  追跡ロスト時に自動復帰(著者は「停止して手動確認」が既定の想定だが、探索目的で有効化)

## 5. 起動(オフライン再生での検証手順)

録画済みbag(例: `ros2_ws/bags/walk_1111`)での検証は、`online_node` +
`graph_based_slam_node`を個別起動し、外部で`ros2 bag play`する構成で確認済み:

```bash
# ターミナル1: RKO-LIO本体
source /opt/ros/humble/setup.bash
source ~/lidarslam_ws/install/setup.bash
ros2 run rko_lio online_node --ros-args \
  -p base_frame:=base_link -p odom_frame:=odom \
  -p lidar_frame:=base_link -p imu_frame:=base_link \
  -p odom_topic:=/rko_lio/odometry \
  -p publish_deskewed_scan:=true \
  -p deskewed_scan_topic:=/rko_lio/frame \
  --params-file /tmp/rko_lio_go2.yaml   # ros2_ws/.../rko_lio_go2.yamlをコピーしたもの

# ターミナル2: バックエンド(ループクロージャは現状不成立。付けたままでも害はない)
ros2 launch lidarslam graph_based_slam.launch.py \
  odom_input:=/rko_lio/odometry cloud_input:=/rko_lio/frame

# ターミナル3: 録画データ再生
ros2 bag play ~/ros2_ws/bags/walk_1111 --clock 100 \
  --topics /utlidar/cloud_base_restamped /go2_state_bridge/imu /tf_static
```

公式の一発コマンド(`ros2 launch lidarslam rko_lio_slam.launch.py bag_path:=...
save_dir:=...`)も使えるが、**疎なデータでは処理が大幅に減速し(終盤は10〜20分で
サブマップ1個分)、5分程度のbagでも数時間かかることがある**。地図(`map.pcd`/
`pose_graph.g2o`)の保存はbag終端イベントかサービス呼び出しに紐づいており、
途中でSIGINT中断すると保存されない点に注意。

## 6. 今後(未着手)

- `real_up.sh`への組込み(新しい追加起動オプションとして、既存の2D経路は維持する方針)
- ループクロージャの却下条件(`max_translation_correction`等)緩和の再検証
- `/go2_state_bridge/imu`のギャップ原因調査
- 実機ライブでの検証(今回はオフラインbag再生のみ)
