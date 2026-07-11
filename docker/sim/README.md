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

# 別ターミナルでコンテナに入って操作する場合
docker exec -it go2-sim bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel

# 終了
docker compose stop
```

## 本体(upstream)との差分・注意点

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
