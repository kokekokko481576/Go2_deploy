# 実機SLAM・AMCLの検証用ツール(#13)

実機Go2で地図作成(slam_toolbox)と自己位置推定(AMCL)を切り分けるために作った道具。
`ros2_ws/bags/` は録画データ置き場としてgitignoreしているが、このフォルダの下記ファイルだけは
`git add -f` で管理している(スクリプト内のパスが `/home/ros/ros2_ws/bags/tools/` 固定のため、場所は動かしていない)。

どれも観測系ノードを起動・購読するだけで、機体を動かす経路(cmd_vel)には触らない。

## 実機ライブ

起動順は必ず **`real_up.sh up` → `live_sept.sh start` → `live_amcl.sh start`**。機体は先に起立させておく
(伏せたままだと `real_up.sh up` の床除去の検定に落ちる)。

```bash
./docker/driver/real_up.sh up
# 地図を作るとき(ang: slam_toolboxの角度探索を±6度に狭める)
docker exec -it arbeit-ros2 bash /home/ros/ros2_ws/bags/tools/live_sept.sh start ang
# 保存した地図でAMCLを動かすとき
docker exec -it arbeit-ros2 bash /home/ros/ros2_ws/bags/tools/live_sept.sh start
docker exec -it arbeit-ros2 bash /home/ros/ros2_ws/bags/tools/live_amcl.sh start
docker exec -it arbeit-ros2 bash -c "source /opt/ros/humble/setup.bash; source /home/ros/ros2_ws/install/setup.bash; python3 /home/ros/ros2_ws/bags/tools/live_fit.py"
```

| ファイル | 役割 |
|---|---|
| `live_sept.sh` | EKF・床除去・slam_toolboxを「9月構成」(`sept/` の設定、床除去 `floor_margin 0.3`)で起動し直す |
| `sept/` | 9/4時点の設定と床除去ノードの控え(ekf.yaml・height_slice_viz.py・pointcloud_to_laserscan.yaml・slam_toolbox_real.yaml) |
| `live_amcl.sh` | slam_toolboxを止め、保存済みの地図(既定 `ros2_ws/map_1118e_ang.yaml`)で地図サーバとAMCLを起動する。AMCLの上書き設定はスクリプト冒頭を参照 |
| `live_fit.py` | AMCLの推定位置にいまのスキャンを置き、壁10cm以内に乗る割合と開始位置テープからのずれを1秒ごとに表示する。記録は `/tmp/live_amcl/fit.tsv` |

`live_sept.sh` のあとに `real_up.sh up` を実行すると、既定の床除去が二重に起動して同じ点群トピックに2つが配信する。
`live_amcl.sh start` は9月構成が動いているとき、既定の方だけを止める。

## 録画の再生(オフライン)

ライブ(既定ドメイン)と並走できる。録画中のbag(.db3)を直接読むと録画プロセスが落ちるので、録画が終わったものに使う。

| ファイル | 役割 |
|---|---|
| `replay_sept_var.sh` | 9月構成でslam_toolboxを再生する。追加引数でslam_toolboxの設定を変えて比べられる。向きの時系列を `<name>.yaw.csv` に残す |
| `replay_amcl_var.sh` | 9月構成でスキャンを作り直し、既存地図の上でAMCLを再生する。結果は `<name>.amcl.tsv` |
| `odom_tf.py`・`yaw_logger.py`・`pose_logger.py`・`amcl_pose_logger.py` | 上の2つが使う補助ノード(TF配信と記録) |
| `slam_truth.py`・`amcl_vs_truth.py` | slam_toolboxの位置を基準に、AMCLの位置ずれを出す |
| `scan_fit.py` | 正解位置なしで、各姿勢のスキャンが地図の壁10cm以内に乗る割合を出す(基準が怪しいときの判定用) |

## 分かっていること(2026-11-19時点)

- 地図が旋回で崩れる原因はslam_toolboxの角度探索。`ang` の設定で、実機365秒の歩行中に向きの飛びが出なかった(11/18、1回のみ)
- AMCLの発散は `robot_model_type` が差動二輪用のままだったことが原因。全方向移動用(Omni)にすると録画の再生では追従した
- AMCLの実機ライブは11/19に起動まで確認。通信の途切れで一周できず、位置誤差の判定はまだできていない
- `go2_localization` の設定ファイル(amcl.yaml・slam_toolbox_real.yaml・床除去の引数)には反映していない。ここでは起動時の上書きだけで試している
