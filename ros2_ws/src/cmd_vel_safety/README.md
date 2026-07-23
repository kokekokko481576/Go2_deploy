# cmd_vel_safety — 速度指令の安全フィルタ中継ノード

Nav2コントローラやテレオペが出す生の速度指令(`cmd_vel_raw`)を受け取り、
速度・加速度の上限クランプと途絶時の自動停止(ウォッチドッグ)をかけてから
ドライバ向けの`cmd_vel`へ中継する、経路追従計画 M1(`docs/計画/経路追従.md` §3.2)の安全機構。

---

## 使い方

### ビルドと起動

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select cmd_vel_safety
source install/setup.bash
ros2 run cmd_vel_safety cmd_vel_safety_node
```

実行可能名は `cmd_vel_safety_node`（`setup.py` の `console_scripts`。ノード名も同じ `cmd_vel_safety_node`）。
launchファイルはこのパッケージには含まれない。単体起動は上記の `ros2 run` で行う。

### 動作確認(手動注入)

別ターミナルから上限を超える指令を流し込み、クランプとウォッチドッグの動きを確認する。

```bash
# 上限(vx=1.0)を超える指令を20Hzで送り続ける
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 5.0}}" -r 20

# 別ターミナルで出力を監視(加速度制限どおりランプアップし1.0で頭打ち)
ros2 topic echo /cmd_vel
```

上の `pub` を止めると、0.5秒後にウォッチドッグが作動して `cmd_vel` がゼロに切り替わる。

### パラメータを変えて起動する例

```bash
# 前後速度の上限を0.6 m/s、ウォッチドッグを0.3sに絞って起動
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args \
  -p max_linear_x:=0.6 -p watchdog_timeout:=0.3

# 出力トピックを /robot1/cmd_vel にリマップして起動(Gazebo等の名前空間向け)
ros2 run cmd_vel_safety cmd_vel_safety_node --ros-args -r cmd_vel:=/robot1/cmd_vel
```

---

## 概要

船内Go2システムの3計画(**自己位置推定 / 経路生成 / 経路追従**)のうち、**経路追従**の
末端に位置する中継ノード。上流(Nav2の`controller_server`やテレオペ)が生成した速度指令を、
そのままドライバへ渡す前に安全側へ整える役割を持つ。

- **速度クランプ**: 各成分を設定した上限内に収める。
- **加速度レート制限**: 直前の配信値からの変化量を制限し、急加速・急停止を抑える。
- **ウォッチドッグ**: `cmd_vel_raw`が一定時間途絶えたら自動でゼロ指令を出し続けて停止する。

無線非常停止(ハードウェア)はこのノードのスコープ外。

### ノード

| 項目 | 値 |
|------|-----|
| ノード名 | `cmd_vel_safety_node` |
| 実行可能名 | `cmd_vel_safety_node` |
| パッケージ種別 | `ament_python` |

### 入出力トピック

| 種別 | トピック | 型 | 説明 |
|------|---------|-----|------|
| 購読 | `cmd_vel_raw` | `geometry_msgs/msg/Twist` | Nav2コントローラ・テレオペ等からの生の速度指令 |
| 配信 | `cmd_vel` | `geometry_msgs/msg/Twist` | クランプ・レート制限・ウォッチドッグ適用後の安全な指令(ドライバがGo2のMove命令に変換する想定) |

いずれもQoSはデフォルト(depth=10)。使用する速度成分は `linear.x` / `linear.y` / `angular.z` の3つ。

---

## 詳細

### パラメータ一覧

| 名前 | 型 | 既定値 | 意味 |
|------|-----|--------|------|
| `max_linear_x` | double | `1.0` | 前後方向速度 `linear.x` の上限 [m/s]（±両方向にクランプ） |
| `max_linear_y` | double | `0.5` | 左右方向速度 `linear.y` の上限 [m/s]（±両方向にクランプ） |
| `max_angular_z` | double | `1.0` | 旋回角速度 `angular.z` の上限 [rad/s]（±両方向にクランプ） |
| `max_linear_accel` | double | `1.0` | 並進(x・y共通)の加速度上限 [m/s²]。レート制限で使用 |
| `max_angular_accel` | double | `2.0` | 旋回の角加速度上限 [rad/s²]。レート制限で使用 |
| `watchdog_timeout` | double | `0.5` | この秒数を超えて `cmd_vel_raw` が来なければ緊急停止 [s] |
| `watchdog_rate` | double | `20.0` | ウォッチドッグの監視タイマ周期 [Hz]（起動時のみ反映） |

### 処理フロー

購読コールバック `_on_cmd_vel_raw`(node.py L41-46)と、定周期タイマ `_on_watchdog_tick`
(L48-63)の2経路で動く。

**1. `cmd_vel_raw` 受信時(通常系)**

1. 受信時刻を `_last_recv_time` に記録し、ウォッチドッグ状態を解除する(L43-44)。
2. `_clamp`(L65-79)で安全化してから `_publish`(L86-89)で配信する。

`_clamp` は次の順で処理する。

- **(1) 速度上限クランプ**: `linear.x` / `linear.y` / `angular.z` をそれぞれ
  `[-max, +max]` に収める(L66-68)。
- **(2) 加速度レート制限**: 直前の配信値 `_last_cmd` からの変化量を、
  `max_accel × dt` 以内に制限する(L70-73)。`dt` は前回配信からの経過秒で、
  最小 `1e-3` s でクランプしてゼロ除算・過大レートを防ぐ(L70)。
  レート制限本体は `_rate_limit`(L81-84):
  `clamp(target, previous ± max_accel·dt)`。
  並進(x・y)は `max_linear_accel`、旋回(z)は `max_angular_accel` を使う。

処理順序が「クランプ → レート制限」である点が重要で、まず絶対上限に収めた目標値に対して、
そこへ至る変化率を制限する。これにより上限を超える指令が来ても、
いきなり上限速度が出るのではなく設定した加速度で滑らかに立ち上がる。

**2. ウォッチドッグ作動時(異常系)**

- 監視タイマ(既定20Hz)が毎周期、最終受信からの経過時間 `elapsed` を評価する(L49-52)。
- `elapsed` が `watchdog_timeout` を超える、または一度も受信していない(`_last_recv_time is None`)場合、
  ゼロTwist(全成分0)を配信する(L54-63)。
- **緊急停止はレート制限を無視して即座にゼロを出す**(L62-63)。安全のため減速ランプは踏まない。
- 警告ログは状態が「未作動→作動」へ遷移した最初の1回だけ出す(`_watchdog_triggered`フラグ、L55-61)。
  途絶が続いてもログをスパムしない。次に `cmd_vel_raw` を受信した時点でフラグが解除され、
  再度途絶えれば改めて1回警告する。

### 設計上の判断

- **加速度上限は x・y で共通(`max_linear_accel`)、z のみ別(`max_angular_accel`)**。
  並進の x/y を区別せず1つのパラメータで扱う簡素化を採用している。
- **`_last_cmd` はレート制限後の実配信値を保持する**(`_publish` 内で更新、L88)。
  レート制限は常に「実際に出した直前値」を基準にするため、指令が飛んでも連続的に追従する。
- **ウォッチドッグ復帰は受信のみをトリガとする**。緊急停止中に `cmd_vel_raw` が届けば
  通常系のコールバックが `_watchdog_triggered` を解除し、そのままクランプ経由で配信が再開する。

---

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
- テレオペからの実際のcmd_vel_rawとの結合(今回は`ros2 topic pub`での手動注入)

Nav2コントローラ(`go2_path_following`の`controller_server`)からの実際の`cmd_vel_raw`との
結合は2026-07-14に確認済み(`launch/controller.launch.py`が`cmd_vel`を`/cmd_vel_raw`に
リマップして本ノードへつなぐ構成。詳細は`go2_path_following/README.md`・Issue #21)。
