# cmd_vel_safety

経路追従計画 M1(`docs/計画_経路追従.md` §3.2)の安全機構。速度・加速度の上限クランプと、
`cmd_vel`が途絶えたときに自動停止するウォッチドッグ(0.5s)を行う中継ノード。
無線非常停止(ハード)はスコープ外。

## I/F

| 種別 | トピック | 型 |
|------|---------|-----|
| 購読 | `cmd_vel_raw` | `geometry_msgs/Twist` (Nav2コントローラ・テレオペ等からの生の指令) |
| 配信 | `cmd_vel` | `geometry_msgs/Twist` (ドライバがGo2のMove命令に変換する想定) |

パラメータ: `max_linear_x`(既定1.0 m/s)、`max_linear_y`(既定0.5 m/s)、`max_angular_z`(既定1.0 rad/s)、
`max_linear_accel`(既定1.0 m/s²)、`max_angular_accel`(既定2.0 rad/s²)、
`watchdog_timeout`(既定0.5s)、`watchdog_rate`(既定20Hz、ウォッチドッグの監視周期)。

処理順序: (1) 速度上限クランプ → (2) 直前配信値からの加速度レート制限。
ウォッチドッグによる緊急停止(ゼロTwist)はレート制限を無視して即座に配信する。

## 使い方

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select cmd_vel_safety
source install/setup.bash
ros2 run cmd_vel_safety cmd_vel_safety_node
```

```bash
# 別ターミナルで速度指令を送る(上限を超える値・ウォッチドッグの動作を確認)
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 5.0}}" -r 20
ros2 topic echo /cmd_vel
```

## 動作確認結果(2026-07-12)

devコンテナ内で確認:

- 上限を超える指令(vx=5.0、上限1.0)を連続publish → 加速度制限どおりランプアップし、
  上限1.0 m/sで頭打ちになることを確認
- `cmd_vel_raw`のpublishを止めると、0.51秒後(ウォッチドッグ閾値0.5s超過)に`cmd_vel`が
  ゼロへ切り替わることを確認。警告ログは状態遷移時に1回だけ出る(スパムしない)

Gazebo連携での確認(devコンテナ=Humble + simコンテナ=Jazzy、別コンテナ・別ROS2ディストロ間):

- 出力トピックを`/robot1/cmd_vel`にリマップして起動し、`cmd_vel_raw`にvx=0.5・wz=0.3を送信 →
  Gazebo上のGo2が実際に前進+旋回し、RViz2上の顎LiDAR点群も連動して変化することを目視確認
  (`ROS_DOMAIN_ID`・`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`が両コンテナで一致していれば、
  Humble⇔Jazzyでも標準メッセージ型は問題なく届く)
- 上記の際、`ros2 topic echo /robot1/cmd_vel`で`invalid data size`/`string data is not
  null-terminated`というCycloneDDSの警告が繰り返し出た(Humble⇔JazzyのXTypes関連の既知の
  相性問題と思われる)。値は正しく届き実際にロボットも正常に動いたため実害は無さそうだが、
  もし今後cmd_vel以外の型でも同様の警告が問題になる場合は要調査
- コマンド停止後、ウォッチドッグ(0.52秒後)でロボットが実際に停止することも確認

未実施:

- 実機での速度応答(遅れ・立ち上がり)込みの検証(計画書3.2-4)
- 無線非常停止(ハードウェア)
- テレオペ・Nav2コントローラからの実際のcmd_vel_rawとの結合(今回は`ros2 topic pub`での手動注入)
