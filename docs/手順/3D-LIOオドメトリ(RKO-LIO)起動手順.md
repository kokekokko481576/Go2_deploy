# 3D LiDARオドメトリ(RKO-LIO)起動手順(Issue #13)

`docs/計画/自己位置推定.md` §1の「3D化(点群 vs 点群地図のマッチング)は精度不足が
確認されてから」の条件が、2026-11-11の実機SLAM検証で満たされたことに伴う記録。
2D AMCL/slam_toolbox自体の変更は行わず、3D LIOは別経路として追加検証した。

**【2026-11-11・最終結論】** `enable_kidnap_relocalization`の扱いを巡って何度か訂正を
重ねた後、**探索コストを抑えた再配置設定(v4)で採用**。バッグ全体(307.8秒)を通して
初めて途切れずオドメトリが出続け、最終位置は開始点から0.05mまで改善した。完全解決では
なく、残課題もある(§6)。経緯は§2.1に記録。

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
| **RKO-LIO**(`rsasaki0109/lidar_slam_ros2`内) | v4設定で採用。バッグ全体を通して完走、最終誤差0.05m。**§2.1・§6参照** |

### 2.1 原因調査の経緯(教訓として記録)

1. **当初「最大ドリフト3.75m」と報告したが誤りだった**: 軌跡を詳細確認した結果、
   `online_node`は歩行開始から約7〜13秒で実質的に追跡を失い、以降は「開始点付近への
   無意味な再配置」を繰り返していただけ(3.75mはドリフト量ではなく再配置の着地点)。
   `voxel_size`を0.1→0.3に粗くする対策も効果なし(むしろ91m規模のあり得ない
   ジャンプが出る、別の悪化パターンに変化)
2. **ソースコード調査で真因を特定**: 原因は`enable_kidnap_relocalization: true`
   だった。追跡失敗時に呼ばれる`try_global_relocalization()`
   (`Thirdparty/rko_lio/rko_lio/core/lio.cpp`)は、過去の全姿勢×複数yaw候補の二重ループで
   候補ごとにフルICPを実行する、非常に重い処理。しかも**registration専用スレッドが
   1本しかない**ため、この間新規フレームを一切処理できずバッファが溢れて古いフレームを
   捨て続け、再配置完了時には溜まったフレームがさらに古くなって再び失敗する、という
   無限ループ(バッファ暴走)に陥っていた。IMUギャップ警告(71件/約5分走行)もこの
   処理遅延の副作用と確認(`lio.cpp`の`motion_priors_from_imu`で確認、bagデータ自体の
   欠落ではない)
3. **「無効化すれば直る」は誤りだった**: `enable_kidnap_relocalization: false`に
   すると暴走は止まったが、代わりに**最初の1回の追跡失敗で以降ずっと復帰不能**に
   なった(IMUのみのdead-reckoningで姿勢が二度とマッチしないレベルまでズレる)。
   upstream RKO-LIOのGo2向け実戦設定例(`bench_report/data/grandtour/.../mgo2.yaml`、
   `enable_kidnap_relocalization: false`)を丸ごと試しても、失敗数が全く同じ(3671件)
   のまま変化せず、この構造を裏付けた
4. **最終解決: 再配置はオンのまま、探索コストだけを約10分の1に削減**
   (`relocalization_yaw_samples: 24→6`、`relocalization_pose_stride: 10→30`、
   `relocalization_max_iterations: 15→8`)。voxel_size等の基本設定は
   upstreamのGo2向け設定(`mgo2.yaml`、`bump_image_registration`=脚ロボット特有の
   「平坦な地面での登録のすべり」対策込み)を採用。結果、**バッグ全体(307.8秒)を
   通してオドメトリが途切れず出続け**、最終位置の誤差は0.05mまで改善(§6に残課題)

**その他の既知の制約**:
- `graph_based_slam_node`によるループクロージャは、今回の疎な顎LiDARデータでは
  **全区間で1件も採用されなかった**(却下673件、採用0件、v4設定とは別の検証時の結果)。
  同じ場所に戻ってもfitness自体は改善することがあるが、その場合は「補正量が0.5mを
  超える」別の安全基準で却下され続ける
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

このGo2向けの上書き設定(v4、採用版)は本リポジトリにコミット済み:
`ros2_ws/src/go2_localization/config/rko_lio_go2.yaml`

要点(詳細な理由はファイル内コメント参照):
- `lidar_timestamps.force_relative: true` — `/utlidar/cloud_base_restamped`の
  `time`フィールド(スキャン内相対時刻)を自動判定できずクラッシュするため明示指定
- `voxel_size: 0.25` / `min_range: 0.7` / `max_range: 100.0` 他多数 —
  upstream RKO-LIOのGo2向け実戦設定例(`bench_report/data/grandtour/.../mgo2.yaml`)を
  ベースに採用。独自に細かくした`voxel_size=0.1`は、むしろ対応点探索を脆くしていた
- `bump_image_registration: true` — 脚ロボット特有の「平坦な地面での登録のすべり」対策
- `enable_kidnap_relocalization: true` + `relocalization_yaw_samples: 6` /
  `relocalization_pose_stride: 30` / `relocalization_max_iterations: 8` —
  再配置は有効だが、探索コストを既定の約10分の1に削減(§2.1参照)

## 5. 起動(オフライン再生での検証手順)

録画済みbag(例: `ros2_ws/bags/walk_1111`)での検証は、`online_node`単体を個別起動し、
外部で`ros2 bag play`する構成で確認済み(ループクロージャが機能しないため
`graph_based_slam_node`は現状付けなくてよい):

```bash
source /opt/ros/humble/setup.bash
source ~/lidarslam_ws/install/setup.bash
ros2 run rko_lio online_node --ros-args \
  -p base_frame:=base_link -p odom_frame:=odom \
  -p lidar_frame:=base_link -p imu_frame:=base_link \
  -p lidar_topic:=/utlidar/cloud_base_restamped -p imu_topic:=/go2_state_bridge/imu \
  -p odom_topic:=/rko_lio/odometry \
  -p publish_deskewed_scan:=true \
  -p deskewed_scan_topic:=/rko_lio/frame \
  --params-file /tmp/rko_lio_go2.yaml   # ros2_ws/.../rko_lio_go2.yamlをコピーしたもの

# 別ターミナルで録画データ再生
ros2 bag play ~/ros2_ws/bags/walk_1111 --clock 100 \
  --topics /utlidar/cloud_base_restamped /go2_state_bridge/imu /tf_static
```

公式の一発コマンド(`ros2 launch lidarslam rko_lio_slam.launch.py bag_path:=...
save_dir:=...`、`offline_node`+`graph_based_slam_node`を両方起動する構成)も使えるが、
**疎なデータでは処理が大幅に減速し(終盤は10〜20分でサブマップ1個分)、5分程度のbagでも
数時間かかることがある**。ループクロージャが機能しない以上、このバッチ経路を使う理由は
現状薄い。地図(`map.pcd`/`pose_graph.g2o`)の保存もbag終端イベント/サービス呼び出しに
紐づいており、途中でSIGINT中断すると保存されない点にも注意。

## 6. 今後(未着手)

- **残課題①**: v4設定でも処理できたフレームは1190/4055(約29%)のみで、バッファ落ちは
  2505件残っている(軽量化前の3576〜3890件からは改善)。登録専用スレッドが1本のみという
  構造上の制約が残っており、マルチスレッド化や探索のさらなる軽量化が必要
- **残課題②**: 走行中(t=50〜98秒付近)に数回、明らかに誤った値(最大54m)へ一瞬飛ぶ場面が
  あった。1〜2サンプル後には自力で妥当な値に復帰する(以前の「一度ズレたら永久に戻らない」
  壊れ方とは異なる)が、`relocalization_min_pose_separation`等の調整や外れ値フィルタで
  抑制する余地がある
- `real_up.sh`への組込み(新しい追加起動オプションとして、既存の2D経路は維持する方針)。
  上記2点が残る間は実機ライブ検証は保留
- `walk_1111_marker`・`walk_1111_sept2`でのv4設定の再現性確認
