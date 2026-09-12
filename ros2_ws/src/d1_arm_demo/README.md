# d1_arm_demo

到達通知を受けて、D1アームを決め打ちの角度へ順に動かすノード。Issue #66・#74。

```
（Nav2 の NavigateToPose 成功）──┐
（marker_approach の到達）    ──┴→ /goal_reached (Bool) → arm_demo_node → d1_arm_controller/commands
```

トリガ源を問わない作りにしてある。どちらも「**成功したときだけ True を1回**」という
同じ契約なので、このノードは誰に呼ばれたかを知らなくてよい。
角度を決めるロジックだけを持ち、送信先はリマップで差し替える（`docs/計画/アーム動作.md` §3-1
の「sim/実機で実装を使い回し、トランスポート層だけ差し替える」方針）。

## 動かす

```bash
ros2 run d1_arm_demo arm_demo_node --ros-args \
  -r goal_reached:=/goal_reached \
  -r arm_command:=/robot1/d1_arm_controller/commands \
  -p step_interval:=4.0
```

simコンテナの中では（`ros2_ws` は読み取り専用マウントでビルドしていないので
`PYTHONPATH` で読む。`marker_approach` と同じやり方）:

```bash
docker exec -d go2-sim bash -c '. /opt/ros/jazzy/setup.bash && \
  export PYTHONPATH=/ros2_ws/src/d1_arm_demo:$PYTHONPATH && \
  python3 -m d1_arm_demo.arm_demo_node --ros-args \
  -r goal_reached:=/goal_reached -r arm_command:=/robot1/d1_arm_controller/commands \
  -p step_interval:=4.0 > /tmp/armdemo.log 2>&1'
```

## パラメータ

| 名前 | 既定 | 説明 |
|---|---|---|
| `step_interval` | `4.0` | ウェイポイント間の待ち時間[s]。**実機では30.0程度にする**（後述） |
| `return_to_neutral` | `true` | 最後に中立姿勢へ戻すか |
| `waypoints` | `[]` | 8個ずつ区切って使う平坦な配列。空なら既定値。ROS2のパラメータは入れ子配列を取れないためこの形 |

関節の並びは `go2_description/config/ros_control.yaml` の `d1_arm_controller` の
`joints:` と同じ順（`j1..j6, gripper_l, gripper_r`。6軸はrad、グリッパー2軸はm）。

## ウェイポイントは1関節ずつ動かすこと（Gazebo実測、2026-09-12）

**複数の関節を1つの指令でまとめて動かすと j1 が可動域上限(±2.36rad)まで走って張り付く。**

| 指令 | 結果 |
|---|---|
| 単軸のみ（j1〜j6 を個別に ±0.4） | 6軸とも指令どおり |
| 2軸同時（j1+j2 / j1+j3 / j1+j5） | 指令どおり |
| 4軸同時 `[0.8, -0.6, 0.4, 0, 0.5, 0, 0, 0]` | **j1 が +2.360 で張り付く**（3回とも再現） |
| 同じ目標を1関節ずつ積み上げ | +0.800/-0.600/+0.400/+0.500 に到達、15秒後も保持 |

`docs/解説/D1アームのGazebo統合のしくみ.md` §8 の「`d1_joint2` に 0.3 を指令すると
可動域上限まで振れて張り付く」（重力無効化で回避済み）と同じ系統の症状が、
多関節同時指令では残っているとみられる。`gz_ros2_control` の
`position_proportional_gain` がロボット全体で共通の1値であることが根にありそうだが
**原因は未特定**（脚の歩行にも効く共通パラメータのため深追いは保留）。

そのため隣り合うウェイポイントの差分は1関節だけにする。2関節以上動く行があると
起動時に警告を出す。実機側（#64）も「30秒に1回の離散コマンド」方針なので
（`docs/計画/アーム動作.md` §4-3）、この作りはそのまま実機へ持っていける。

## 既定のウェイポイントは暫定値

中立 → j1でベースを振る → j2で肩 → j3で肘 → j5で手首を下へ、の5点。
**sim での見た目合わせでしかない。** 搭載位置の静的TFも関節の回転軸も実機未照合なので
（`docs/計画/アーム動作.md` §5）、実機ではこの数値をそのまま使わないこと。

## 走行中はアームを中立に固定する

2D footprint は静的な矩形で、動くアームの可動範囲をカバーしていない
（`docs/解説/D1アームのGazebo統合のしくみ.md` §8）。ナビゲーション中は中立、
停止後にのみ動かす、という前提を崩さないこと。このノードは到達通知を受けてから
動き出すので、その前提に沿っている。

## 通し確認（2026-09-12、Gazebo）

`marker_approach` と繋いで、マーカー検知→接近→真横旋回→アームまで通っている。

```bash
cd docker/sim && SIM_ENABLE_NAV2=false docker compose up -d
# arm_demo_node を起動（上記）してから
./docker/sim/tools/sim_up.sh --place --go
```

接近の到達「位置誤差 58mm、方位 -4.2度、法線ずれ -5.3度、2周目」の直後に
到達通知が渡り、アームが5点を順に辿って目標姿勢で静止した。
