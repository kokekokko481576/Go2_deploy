# docker/sim/tools — Gazebo真値ツール(Issue #43、sim限定・後で丸ごと削除する前提)

## これは何か

Gazeboの**物理演算そのもの**(神視点、シミュレーション限定)からロボットの位置を
取得して`/gz_ground_truth/pose`にpublishするだけのツール。#14/#36/#23での調査で、
「壁接触時に脚オドメトリが滑りを検知できず、AMCLの推定位置が真値と1m近くズレる」
ことが判明し、それ以降の検証で毎回`gz topic -e -t /world/default/pose/info`を手で
叩いて確認していた。それを自動化した。

## 真値(このツール) と 推定値(本番の自己位置推定)の違い — 混同しないこと

| | トピック | 出所 | 実機での有無 |
|---|---|---|---|
| **真値**(このツール) | `/gz_ground_truth/pose`(`geometry_msgs/msg/PoseStamped`、`frame_id: map`) | Gazebo物理演算(`gz topic -e -t /world/default/pose/info`) | **無い**(sim限定) |
| **推定値**(本番) | `/go2_localization/tf`(`map→odom→base_link`)、`/go2_localization/amcl_pose`等 | 自作EKF/AMCL(`go2_localization`) | 実機でも動く本番システム |

- **どのノードも`/gz_ground_truth/pose`をsubscribeしてはいけない**(制御・計測ロジックに
  真値を混ぜない)。使うのは「人間が真値と推定値を見比べる」「計測スクリプトが
  検証用にログへ両方書き出す」場合のみ
- 変数名・トピック名には必ず`gz_ground_truth`(このツール)か`go2_localization`
  (本番推定)のどちらかを含め、どちらの値か一目で分かるようにする
- **実機移行時はこの`docker/sim/tools/`ディレクトリを削除するだけで良い**設計にしている
  (`ros2_ws`内のどのパッケージもこのディレクトリ・トピックを参照しない)

## 使い方

devコンテナ・simコンテナ起動後、ホスト側で:

```bash
./docker/sim/tools/start_ground_truth.sh   # 起動
./docker/sim/tools/stop_ground_truth.sh    # 停止
```

比較する場合の例(devコンテナ側):

```bash
# 真値
docker exec go2-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /gz_ground_truth/pose --once --no-daemon"

# 推定値(map->base_link)
docker exec arbeit-ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && \
  ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/go2_localization/tf"
```

## 実装メモ

`ros_gz_bridge`で`gz.msgs.Pose_V`(`/world/.../pose/info`)を`tf2_msgs/msg/TFMessage`へ
ブリッジすると、このJazzy環境ではエンティティ名(`name`)が失われて`child_frame_id`が
空文字になる既知の制限があり(実際に確認済み)、名前でフィルタできず使えなかった。
そのため`gz topic -e`の標準出力を直接テキストパースする方式にした(`gz`コマンドの
利用可否に依存するため、`ros2_ws`のパッケージにはせずsimコンテナへ`docker cp`する
単体スクリプトにしている)。

Pose更新は`/world/default/pose/info`の配信レートに追従するため約80〜90Hz出る
(スロットリングはしていない。デバッグ用途で問題にならない範囲)。

タイムスタンプは`Pose_V.header.stamp`(Gazeboのsim時刻)をそのまま使う(受信時のROS時刻
ではない)。他のsim時刻同期ノードとの時刻ベース比較で誤差要因にしないため。

**単一ロボット構成が前提**。`robots.yaml`でrobot2以降を有効化するなどでspawn時に
`-allow_renaming true`が働きエンティティ名が変わった場合、`--entity-name`(既定
`robot1_my_bot`)と一致せず**エラーも警告も出さず何も配信しなくなる**ことがある。
これを検知するため、`--stale-warn-sec`(既定5秒)間pose未取得なら警告ログを出す。
