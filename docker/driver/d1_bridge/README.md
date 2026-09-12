# d1_bridge — D1-Tアームの実機トランスポート層

Issue #64。`d1_arm_demo`（sim/実機共通の「角度を決めるロジック」）の**送信先だけを
差し替える**ための層（`docs/計画/アーム動作.md` §3-1）。

```
d1_arm_demo ──arm_command(Float64MultiArray, rad)──▶ arm_bridge_node
                                                        │ JSONへ変換(度)
                                                        ▼
                                      /arm_Command (unitree_arm/ArmString)
                                                        │ ROS2 が rt/ を付ける
                                                        ▼
                                                  rt/arm_Command → D1-T
```

2パッケージ入っている。

| パッケージ | 中身 |
|---|---|
| `unitree_arm` | DDS の型名を SDK と一致させるためだけのメッセージ定義（`ArmString` / `PubServoInfo`） |
| `d1_arm_bridge` | `arm_command`(rad) を D1 の JSON コマンド(度)へ変換して送るノード |

## なぜ unitree_sdk2 を使わないのか

**1つのプロセスで rclcpp と unitree_sdk2 を両方リンクすると、2026-08-28 に実機で
踏んだ `free(): invalid pointer` がそのまま再発する。** ROS2 Humble の CycloneDDS と
`/usr/local/lib` の unitree_sdk2 純正 CycloneDDS が同名で ABI 違いのため、
`LD_LIBRARY_PATH` で先に見つかった方が読まれて壊れる（`d1_sdk/run.sh` が
`/usr/local/lib` を優先させているのはこの対策）。

それを避けられるのは、**D1 のコマンド経路がそのまま ROS2 の経路と一致している**から。

- SDK のサンプルは CycloneDDS の生トピック `rt/arm_Command` に
  `unitree_arm::msg::dds_::ArmString_`（文字列1個）を publish している
- **ROS2 のトピック `/arm_Command` は DDS 上では `rt/arm_Command` になる**
  （`rt/` は ROS2 が付ける接頭辞。Go2 本体の `/api/sport/request` ↔
  `rt/api/sport/request` と同じ）
- ROS2 のパッケージ `unitree_arm` のメッセージ `ArmString` は DDS 型名
  `unitree_arm::msg::dds_::ArmString_`、フィールド `data_` を生成する

型名・フィールド名・トピック名の3つが一致するので、ROS2 のパブリッシャが
そのまま機体のサブスクライバとマッチする。**だからパッケージ名もメッセージ名も
フィールド名も変えてはいけない。**

## `rt/` プレフィックス問題の答え（#63 の宿題）

Koga さんが #63 で挙げた矛盾は、SDK の実物を読んで決着した。**コマンドとフィードバックで
違う**というのが答え。

| 向き | トピック | `rt/` |
|---|---|---|
| コマンド（PC → アーム） | `rt/arm_Command` | **付く** |
| フィードバック（アーム → PC） | `current_servo_angle` / `arm_Feedback` | **付かない** |

SDK 同梱サンプルがこの通りに書かれており、2026-08-28 に**このPCの個体で6本とも
実機動作した**（`~/ダウンロード/D1アーム_実機検証_作業報告.pptx`）。Caltech の報告が
`rt/arm_Feedback` だったのは、おそらくファームウェアの版差。

**結果として、フィードバックは ROS2 からは購読できない。** ROS2 は必ず `rt/` を付けるため、
`rt/` なしの生トピックには手が届かない。`PubServoInfo.msg` は型定義だけ置いてあるが、
このノードは購読していない。角度を読みたいときは当面 `d1_sdk/run.sh get_arm_joint_angle`
を使う。もしファームが `rt/` 付きで配信する個体なら、ROS2 から
`/arm_Feedback` `/current_servo_angle` として購読できるようになる。

## コマンドの形式（SDK のサンプルから読み取った実物）

| funcode | 用途 | data |
|---|---|---|
| 1 | 単一関節 | `{"id":5,"angle":60,"delay_ms":0}` |
| 2 | 複数関節 | `{"mode":1,"angle0":0,...,"angle6":0}` |
| 5 | 有効化/無効化 | `{"mode":0}` |
| 7 | ゼロ姿勢へ | (なし) |

**角度の単位は度。** `restore_initial_pose.cpp` が `91.4` `-89.3` を送っている。
`arm_command` はラジアン（sim と同じ）で受けるので、ノード内で度へ変換する。
`mode` は 0 が「10Hzデータの小さい平滑化」、1 が「軌道用の大きい平滑化」。

## 動かす

```bash
# driverコンテナで。まずは dry_run のまま（既定）で JSON を見る
ros2 run d1_arm_bridge arm_bridge_node --ros-args -r arm_command_out:=/arm_Command

# 実機へ出すとき
ros2 run d1_arm_bridge arm_bridge_node --ros-args \
  -r arm_command_out:=/arm_Command -p dry_run:=false -p min_command_interval:=25.0
```

`d1_arm_demo` 側は送信先を変えるだけ:

```bash
ros2 run d1_arm_demo arm_demo_node --ros-args \
  -r goal_reached:=/goal_reached \
  -r arm_command:=/arm_command \
  -p step_interval:=30.0
```

## パラメータ

| 名前 | 既定 | 説明 |
|---|---|---|
| `dry_run` | **`true`** | true の間は JSON をログに出すだけで送らない。**実機へ出すときだけ false にする** |
| `mode` | `1` | funcode 2 の平滑化モード |
| `address` | `1` | |
| `min_command_interval` | `1.0` | 送信間隔の下限[s]。**実機では25〜30にする**（後述） |
| `joint_signs` | `[1,1,1,1,1,1]` | 関節の回転方向。**実機で目視照合してから埋める** |
| `joint_offsets_deg` | `[0,0,0,0,0,0]` | 同上、原点のずれ |
| `gripper_open_m` / `gripper_closed_deg` / `gripper_open_deg` | `0.033` / `0` / `0` | simの prismatic 2軸[m] を D1 の `angle6` へ割り当てる。**対応は推測のまま** |

## 安全のための作り

- **`dry_run` が既定で true。** 実機が動く側の既定を「動かない」にしてある
- **可動域を超える角度はクランプする**（J1/J4/J6 ±135度、J2/J3/J5 ±90度。公称スペック）
- **`min_command_interval` で送信間隔に下限**を設ける。D1 は連続コマンドで数分後に
  無応答化するという他ラボの報告があり（`docs/計画/アーム動作.md` §4-3）、
  低頻度の離散コマンドが基本方針。実機では 25〜30 秒にすること
- **`seq` は毎回変える。** Go2 本体では `header.identity.id` を固定したまま同じ内容を
  送り続けると機体が重複とみなして無視する、という実機実測がある（前進効率 40%→83%）。
  D1 で同じ挙動をするかは未確認だが、固定にする理由が無いので増やしている

## アームが届いたら最初に確かめること

1. **dry_run のまま**起動し、`d1_arm_demo` を繋いで JSON が期待どおり出るか見る
2. `ros2 topic info /arm_Command` でパブリッシャが立っているか、
   機体側のサブスクライバとマッチしているか（`--verbose` で相手が見える）
3. `dry_run:=false` にして `zero_pose` に True を1回（funcode 7）。**まずゼロ姿勢だけ**
4. 通ったら単軸を1つだけ小さく動かし、**回転方向を目視で照合**して `joint_signs` を埋める
5. `d1_sdk/run.sh get_arm_joint_angle` を別端末で回して、指令と実測が合うか確認
6. 低頻度コマンドで5分間、無応答化しないことを確認（#63 A2）

## うまくいかなかったら

ROS2 から直接マッチしない場合（型ハッシュ・QoS・ファーム差）の代替は、
**プロセスを分ける**こと。ROS2 ノードは JSON を FIFO か Unix ソケットへ書き、
unitree_sdk2 だけをリンクした小さな C++ プロセスがそれを読んで
`rt/arm_Command` へ publish する。ABI 衝突は起きないが、プロセスが1つ増える。
`docker/driver/d1_sdk/src/multiple_joint_angle_control.cpp` がほぼそのまま雛形になる。

## 実機なしで検証済み（2026-09-13）

**機体と同じ立場に立つ購読プログラム（`tools/dds_probe.cpp`、unitree_sdk2 で生の
`rt/arm_Command` を `unitree_arm::msg::dds_::ArmString_` として購読）が、
ROS2 側のブリッジが送った JSON をそのまま受信することを確認した。**
アームは要らない。同じPC上で2プロセス立てるだけで確かめられる。

```
[probe] 受信 #1: {"seq":1,"address":1,"funcode":2,"data":{"mode":1,"angle0":89.95,"angle1":0.0,...}}
[probe] 受信 #2: {"seq":2,"address":1,"funcode":2,"data":{"mode":1,"angle0":89.95,"angle1":68.75,...}}
[probe] 受信 #3: {"seq":3,"address":1,"funcode":7}
[probe] 受信 #4: {"seq":4,"address":1,"funcode":2,"data":{"mode":1,"angle0":0.0,"angle1":90.0,...}}
```

確認できたこと: トピック名(`/arm_Command` → `rt/arm_Command`)・DDS型名・フィールド名が
SDK と一致すること、`seq` が毎回増えること、`zero_pose`(funcode 7)が通ること、
可動域超過（J2 に 2.0rad = +114.6度）が ±90度 にクランプされること。

**残るのは実機固有の部分だけ**: 機体が実際にこの JSON を受理するか、
関節の回転方向、グリッパー `angle6` の対応。

### 検証のやり方

```bash
docker compose -f docker/driver/compose.yaml run --rm -d --name d1test driver sleep 3600
docker exec -d d1test bash -c 'source /setup_dds.sh; /root/d1_bridge_tools/run_probe.sh 45 > /tmp/probe.log 2>&1'
docker exec -d d1test bash -c 'source /setup_dds.sh; ros2 run d1_arm_bridge arm_bridge_node \
  --ros-args -r arm_command_out:=/arm_Command -p dry_run:=false -p min_command_interval:=0.5 > /tmp/bridge.log 2>&1'
docker exec d1test bash -c 'source /setup_dds.sh; ros2 topic pub --once /arm_command \
  std_msgs/msg/Float64MultiArray "{data: [1.57, 1.2, 0, 0, 0, 0, 0, 0]}"'
docker exec d1test cat /tmp/probe.log
```

**`go2-sim` を止めてから行うこと。** sim は同じ ROS_DOMAIN_ID=0 に多数のノードを立てるため、
CycloneDDS の participant index を使い切って
`Failed to find a free participant index for domain 0` で両方とも起動できない。
