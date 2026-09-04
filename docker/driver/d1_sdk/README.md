# d1_sdk — D1-T アームの疎通用サンプル

対象: Issue #63「D1-T SDK疎通確認+低頻度コマンドでの信頼性検証」、
方針は [`docs/計画/アーム動作.md`](../../../docs/計画/アーム動作.md)。

D1-Tアームは Go2 の Sport Mode API とは**別系統**で、`unitree_sdk2` のDDSチャネル層を
直接使う(ROS2のメッセージは介さない)。そのため `ros2 run` ではなく、
ビルド済みバイナリを `run.sh` 経由で実行する。

## 使い方

```bash
cd docker/driver
docker compose up -d

# 実行ファイル一覧
docker compose exec driver /root/d1_sdk/run.sh

# フィードバック受信(関節角度・電源状態)。Ctrl-Cで抜ける
docker compose exec driver /root/d1_sdk/run.sh get_arm_joint_angle

# ゼロ姿勢へ戻す(1コマンドだけ送って終了)
docker compose exec driver /root/d1_sdk/run.sh arm_zero_control
```

イメージビルド時に `/root/d1_sdk/build` へビルド済み。ソースを直したら
`docker compose build` でイメージを作り直す。

## `run.sh` を経由しなければならない理由(重要)

`unitree_sdk2` は**自前でビルドしたCycloneDDS**を `/usr/local/lib` に置く。これは
ROS2 Humble が使うCycloneDDS(`/opt/ros/humble/lib`)と**ABIが違う**。
ROS2側が先に解決されると `free(): invalid pointer` で落ちる。

対処として、

- Dockerfile では **`ldconfig` を実行していない**。実行すると `/usr/local/lib` の
  CycloneDDSが ld.so のキャッシュに載り、ROS2のノード側がこちらを掴んで落ちる
- `run.sh` が `LD_LIBRARY_PATH=/usr/local/lib` を先頭に付けてから実行する。
  D1のバイナリだけがSDK同梱のCycloneDDSを見る

バイナリを直接叩くと(`/root/d1_sdk/build/get_arm_joint_angle`)この対処が効かないので、
必ず `run.sh` を通すこと。

## サンプル一覧

いずれも `rt/arm_Command` に1コマンドをpublishして終了する(`get_arm_joint_angle` を除く)。
DDSのディスカバリ完了前にWriteすると届かないため、送信前に1秒待つ作りになっている。

| 実行ファイル | funcode | 内容 |
|---|---|---|
| `get_arm_joint_angle` | — | フィードバック購読(サーボ角度・`arm_Feedback`)。Ctrl-Cまで動き続ける |
| `arm_zero_control` | 7 | ゼロ姿勢へ戻す |
| `joint_enable_control` | 5 | 関節の有効化/放電(`mode`指定) |
| `joint_angle_control` | 1 | 単一サーボの角度指令(`id`/`angle`/`delay_ms`) |
| `multiple_joint_angle_control` | 2 | 全7サーボ(6軸+グリッパー)の角度をまとめて指令 |
| `restore_initial_pose` | 2 | 実測した初期姿勢へ戻す(全7サーボ) |

## 未確認・注意点

- **`get_arm_joint_angle` の購読トピック名に `rt/` が付いていない**
  (`current_servo_angle` / `arm_Feedback`)。コマンド送信側は `rt/arm_Command` と
  `rt/` 付きなので不揃いになっている。`docs/計画/アーム動作.md` §4-4 は
  「feedbackトピック名は `rt/arm_Feedback`。`arm_Feedback` とだけ書くと購読できない、
  というのが他ラボでの実際のハマりどころだった」としており、**このままでは
  feedbackを受けられない可能性が高い**。SDK同梱のまま持ち込んでおり実機での
  受信確認が取れていないため、意図的に変更していない。実機で最初に確認すること
- **連続コマンドを送り続けない**。他ラボの報告では数分でアームが無反応になり、
  「30秒に1回程度の低頻度コマンド」なら約5分安定した。Issue #63 の完了条件も
  この低頻度パターンでの確認になっている。詳細は `docs/計画/アーム動作.md` §4-3
- `ChannelFactory::Instance()->Init(0)` はNICを指定していない。driverコンテナでは
  `CYCLONEDDS_URI`(=`GO2_NIC`で生成)が効くため、アームと繋がるNICを `GO2_NIC` に
  指定する必要があるかどうかは実機で確認する
- 本ディレクトリはUnitree提供のサンプルが元。`src/msg/` の型定義は生成物をそのまま
  持ち込んでいる
