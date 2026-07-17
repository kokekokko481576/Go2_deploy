# simコンテナ（Gazebo + Go2モデル + Nav2）

対象: `docs/docker要件定義.md` §4.1「sim: go2_ros2_sim_py の流用方針」、段階導入1。
外部リポジトリ [go2_ros2_sim_py](https://github.com/abutalipovvv/go2_ros2_sim_py) を
`external/go2_ros2_sim_py` に git submodule として取り込み、本体は変更せず利用する。

## できること

- Gazebo(Ignition/Gazebo Harmonic) 上で Go2 モデルを歩行・旋回させる(独自Python IK歩容)
- Nav2フルスタック(planner/controller/behavior/smoother/bt_navigator)込みで起動
- カメラ・IMU・2D LiDAR(`/robot1/scan`)がGazebo→ROS2ブリッジ経由で流れる

## 使い方

```bash
cd docker/sim
docker compose build sim   # 初回のみ。数GBダウンロード+colcon build、15〜30分程度
xhost +local:docker        # GUI表示のため(ホストで一度)
docker compose up -d
docker logs -f go2-sim     # 起動ログ確認(Gazebo GUIウィンドウも表示される)

# 別ターミナルでキーボード操作する場合(下記 or ./teleop.sh のどちらでも可)
# zshなら~/.zshrc起動時にROS2/ワークスペースを自動sourceするので手動sourceは不要
docker exec -it go2-sim zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel

# bashで入る場合は/ros_entrypoint.shを経由しないためROS環境が未sourceの状態になり、
# 都度sourceが必要な点に注意
# docker exec -it go2-sim bash
# source /opt/ros/jazzy/setup.bash
# source "$WORKSPACE_DIR/install/setup.bash"

# または簡単に:
./teleop.sh

# 終了
docker compose stop
```

**注意**: Gazebo GUIの「Reset」ボタン(World > Reset / 左下の巻き戻しアイコン)を押すと、
ロボット(`robot1_my_bot`)はワールドファイル(`cafe.world`)に含まれず起動後に動的にspawnされた
存在のため、**消えたまま復活しない**。ロボットが消えた場合は`docker compose restart`で
launchシーケンス(spawn含む)をやり直す必要がある。

## 本体(upstream)との差分・注意点

- **`external/go2_ros2_sim_py` は自分のfork(`kokekokko481576/go2_ros2_sim_py`、public)を参照**。
  理由: 顎3D LiDAR追加(下記)がxacro/launch本体の編集を要し、他コンテナのようなcompose override
  では対応できないため。upstreamへのPR送付は意図していない。submoduleの`upstream`リモートに
  元リポジトリを残してあり、upstream更新の追従自体は今後も可能
- **`enable_nav2`起動引数を追加(2026-07-17、Issue #35)**: `gazebo_sim/launch/launch.py`と
  `gazebo_multi_nav2_world.launch.py`に`enable_nav2`引数(既定`true`=従来挙動)を追加した。
  `enable_nav2:=false`でupstream本家のNav2スタック一式(map_server/amcl/planner/controller/
  behavior/smoother/bt_navigator/initialpose投入)を起動せず、Gazebo+ロボットspawn+歩容+
  odom/EKF+センサブリッジのみになる。dev側の自作パイプライン(`go2_path_following`)による
  GATE1定量計測時に、upstream側の`controller_server`(1個)+`behavior_server`(5個)が
  `/robot1/cmd_vel`へ持つpublisherを排除して計測汚染を防ぐための口。あわせて`enable_rviz`も
  `launch.py`からパススルーできるようにした。compose.yamlの`SIM_ENABLE_NAV2`/`SIM_ENABLE_RVIZ`
  環境変数から渡せる(`SIM_ENABLE_NAV2=false docker compose up -d`)。使い方は
  `go2_path_following/README.md`「GATE1計測時のトピック確認」を参照
- **顎(chin)搭載3D LiDARを追加(2026-07-12)**: `go2_description/xacro/robot.xacro`に
  `chin_lidar_frame`リンク、`gazebo.xacro`に垂直スキャン付き`gpu_lidar`センサを追加し、
  `gazebo_multi_nav2_world.launch.py`のros_gz_bridgeに`PointCloud2`のブリッジ行を追加した。
  トピックは`/robot1/chin_lidar/scan/points`(`sensor_msgs/msg/PointCloud2`)。
  **搭載位置(base_link相対 x=0.29, z=-0.06)・下向きピッチ・垂直FOV(16ch, ±15°)は
  実機スペックの根拠が無い仮値**(未使用の在庫URDFの`Head_lower`を流用)。実機の顎LiDAR型番・
  搭載位置が確定次第、`go2_description/xacro/gazebo.xacro`の`chin_lidar`センサ定義を要更新
- **ROS2ディストロはJazzy固定**（go2_ros2_sim_py自体がJazzy前提。README記載）。
  プロジェクト全体のHumble固定(unitree_ros2の都合、`docker/`のdevコンテナ)とは別に、
  simコンテナだけ独立してJazzy+Gazebo Harmonicで動かす。cmd_vel等の標準メッセージ型は
  ディストロを跨いでも相互運用できるため、Nav2連携の契約自体には支障ない
- **upstream純正の `docker/Dockerfile` はそのまま使わず、本ディレクトリに独自のDockerfileを用意**。
  理由: upstream版は `colcon-cache` がワークスペースの `.git` HEAD参照を前提にしているが、
  本プロジェクトではgo2_ros2_sim_pyを **git submodule** として取り込んでおり、submodule内の
  `.git` はコンテナに複製すると無効な参照になり `colcon-cache` が
  `Ref 'HEAD' did not resolve to an object` で失敗する。そのため colcon-cache を使わず
  素の `colcon build --symlink-install` のみを行う(本体のソース・Dockerfileそのものは変更していない)
- **GPU: upstream は NVIDIA GPU 必須構成**(`docker/compose.yml` の `deploy.resources.reservations.devices: driver: nvidia`)。
  本プロジェクトの compose.yaml ではこれを使わず、要件定義R4どおり iGPU(`/dev/dri`)渡し込みに
  差し替えている。NVIDIA環境で使う場合は `compose.yaml` 内のコメントを参照

## 動作確認結果(2026-07-12、開発PC: Ubuntu22.04 / AMD Ryzen 5 8640U / Radeon 760M iGPU)

- Gazebo(Harmonic) + Go2モデル(`robot1_my_bot`) + Nav2フルスタックの起動を確認
- リアルタイム係数(`gz topic -e -t /stats`): 0.5〜1.4で推移、平均概ね1.0前後 → **iGPUでも実用速度**
- コンテナのメモリ使用量: 約1.4GB/14.9GB(9%)、CPU: 12論理コア中6コア分程度
- `docs/docker要件定義.md` §6 未決事項「go2_ros2_sim_pyがiGPUで実用速度か」は本確認により解消

未検証:

- 実際のNav2目標到達・障害物回避などシナリオレベルの検証(今回は起動確認とRTF計測のみ)
- 長時間稼働時の安定性
- ホストのメモリ・ディスクが逼迫している状態での再現性(検証時はディスク17GB・メモリ余裕を確保した状態で実施)

## 顎LiDAR動作確認結果(2026-07-12)

- ヘッドレス(`gz sim -s`)でのGazebo起動 + `chin_lidar`センサのロードをエラーなく確認
  (既存のlaser/imu/cameraと同一の`gz_frame_id`警告のみ。新規の警告・エラーは発生していない)
- `ros_gz_bridge`が`/robot1/chin_lidar/scan/points`(`PointCloud2`⇔`gz.msgs.PointCloudPacked`)の
  ブリッジを正常に生成することを確認
- 実デスクトップでの`docker compose up -d`(GUIあり)でRViz2にPointCloud2表示
  (`/robot1/chin_lidar/scan/points`)を追加し、点群が実際に流れていることを目視確認。
  垂直16chが下向きピッチで地面に多重の円弧を描く、意図通りの3D LiDARらしい点群形状になっていた

未実施:

- 搭載位置・ピッチ・FOVの実機スペックとの整合(上記「本体との差分」節のとおり全て仮値)
- 水平FOVは既存2D LiDARの設定(360°全周)を暫定的に引き継いだまま(実機確定時に前方限定等へ見直す想定)
