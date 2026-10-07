# tools/inspection — ビード検査の一連の動作（実機、#74）

マーカーで部材の横へ近づき、伏せて D1 アームで撮影し、収納して起立するまでを1コマンドで進める。

    マーカー接近 → 伏せる（StandDown） → アーム展開 → 撮影（手首を振って3枚） → 収納 → 起立（StandUp）

2026-09-29 に実機で5回通した（`--auto` の本番を含む）。

## いまの前提（Go2_deploy にまだ取り込めていない部分）

**このディレクトリはホスト側の作業環境 `~/marker_detection` で動かしたものをそのまま置いている。**
次のものは Go2_deploy に無いので、実機で動かすにはホーム側の環境が要る。取り込みは別の作業にする。

| 使っているもの | 場所 |
|---|---|
| マーカー検出ノードと、実機用の接近の起動 `approach_real.launch.py`（`go2_cmd_vel_bridge`） | `~/marker_detection/ros2_ws` |
| D1 アームの操作 `run.sh arm_server`（既定）/ `set_joints` / `get_arm_joint_angle` | `~/d1_sdk`（`--d1-run` で変更可）。`arm_server`・`set_joints` は SDK に無いので `d1_sdk_extra/` から入れる（下記） |
| RealSense の撮影 `rs_snap`（Jetson 側でビルド） | Jetson の `~/riku_rs/`（`rs_snap_install.sh` で入れる） |
| ログ・写真の保存先 | `~/marker_detection/logs/` |

sim の通し試験（`sim_world_node.py`・`sim_inspect_layout.py`）は、同じディレクトリに
`sim_approach.py`（このリポジトリでは `ros2_ws/src/marker_approach/tools/`）がある前提。

## ファイル

| ファイル | 役割 |
|---|---|
| `inspect_run.py` | 一連の動作を進める本体。温度・アーム到達・接近結果で止まる |
| `inspect_poses.json` | アームの姿勢（収納、立ち・伏せそれぞれの展開経路、手首を振る角度） |
| `monitor_temp.py` | 後脚股関節の温度を見張る（読むだけ）。70℃超で赤点滅して転倒した（9/28） |
| `env_real.sh` | 実機用の端末環境（CycloneDDS・enp2s0）。各端末で最初に source する |
| `rs_snap.sh` / `rs_snap.cpp` / `rs_snap_install.sh` | Jetson の RealSense で撮って PC に持ってくる。`--depth 848x480` で近い距離の深度も取れる |
| `sim_world_node.py` | 通し試験用の仮想世界（機体・検出器・温度・伏せ／起立の高さ） |
| `sim_inspect_full.sh` | 実機と同じ `approach_node` と `inspect_run.py` を、代役相手に実機なしで通す |
| `sim_inspect_layout.py` | 出発位置のばらつきに対する止まる位置の統計（ROS 不要） |
| `sim_fakes/` | D1 アームと撮影の代役。`render_snap.py` は代役のアームの角度から部材の深度画像を合成する（`SIM_RENDER=1`） |
| `aim_correct.py` | 撮影位置の補正。深度でベース板と縦板の交線（隅肉の根元）を見つけ、アームを直す（`--aim-correct`） |
| `merge_steps.py` | 展開経路のうち、まとめて送っても干渉しない段を探して `deploy_fast` を作る |
| `aim_pose.py` / `arm_reach_study.py` | アームの運動学と干渉のモデル（実機で先端位置が2〜6mmで一致）。上の2つが使う |
| `d1_sdk_extra/` | `~/d1_sdk` に足すプログラム: `arm_server.cpp`（常駐して送信と角度の読み取り）・`set_joints.cpp` |

## 使い方（実機）

```bash
# 端末1: 温度（後脚股関節 50℃以下で始める）
python3 monitor_temp.py
# 端末2: 接近制御（有効化されるまで待機）
ros2 launch go2_cmd_vel_bridge approach_real.launch.py dry_run:=false use_normal:=false
# 端末3: 一連の動作
python3 inspect_run.py --approach marker --posture lie --auto --hold 5
```

何も動かさず流れだけ見るなら `--dry-run`。1枚だけ撮るなら `--no-bracket`。

### 2026-10-07 に足したもの（いずれも実機未確認。sim と過去の実機データで確認）

- **収納を真後ろへ**: `stow` の angle0 を 68.7 → 0。腕が機体の軸に沿い、左右の重心の偏りがほぼ無くなる。
  起動時に収納姿勢と違えば、根元の旋回だけを合わせる（既存の動作）
- **アームとのやり取りを常駐させる**（既定 `--arm-io server`）: 1段ごとに `set_joints` と `get_arm_joint_angle` を
  起動していたため1段約5秒かかっていた（10/1 実機: 伏せの展開9段で68秒）。2秒動き出さなければすぐ再送する。
  `--arm-io cli` で以前の方式
- **`--fast-path`**: 伏せの展開を9段 → 3段（`deploy_fast`）。各関節がどの順に着いても干渉しない段だけを
  `merge_steps.py` でまとめた。**複数関節の同時指令は実機で未試験**なので既定では使わない
- **`--aim-correct`**: 撮影姿勢に着いたら試し撮りし、根元が画面の中央・`--aim-dist`（0.22m）に来るよう最大 `--aim-iter` 回直す。
  手首を振る3枚は直した姿勢を中心に撮り、収納は補正を逆にたどってから。**干渉の判定に部材は入っていない**。
  動きを1関節ずつにするなら `--aim-single-joint`
- **グリッパー**: 10/1 にカメラを D405 に交換して挟み直したので `gripper` は -12.6（旧 D435i は 13.2）

### `~/d1_sdk` への追加（arm_server・set_joints）

```bash
cp d1_sdk_extra/*.cpp ~/d1_sdk/src/
cat >> ~/d1_sdk/CMakeLists.txt <<'CMAKE'
add_executable(set_joints src/set_joints.cpp src/msg/ArmString_.cpp)
add_executable(arm_server src/arm_server.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
CMAKE
cd ~/d1_sdk/build && cmake .. && make set_joints arm_server
```

（`set_joints` が既にあるなら1行目の `add_executable` は不要）

### sim での確認

```bash
./sim_inspect_full.sh                                                  # 既定
SIM_RENDER=1 SIM_ROOT_DY=0.06 D1_FAKE_DROP=1 INSPECT_ARGS="--fast-path --aim-correct" ./sim_inspect_full.sh
```

## 分かっている課題

- **止まる位置が回ごとに±7cm ばらつく。** 直進中に最大約14度左へ流れる回と流れない回がある。
  根本の対策（直進中の向き補正）は未実装。応急として手首を -82/-69.5/-57度に振って3枚撮る。
  撮影位置のずれは `--aim-correct` で直す（sim の閉ループ60通りで全て光軸から3度以内。実機は未確認）
- 立ったままアームを出し続けると後脚股関節が過熱する。デモは伏せてからアームを出す流れにしている
