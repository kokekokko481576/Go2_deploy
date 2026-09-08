# D1アームのGazebo統合のしくみ(やさしい解説)

対象読者: Issue #65(URDF移植)・#66(Gazebo一気通貫デモ)にこれから触る人向け。
「どのリポジトリのどのデータを引っ張ってきて、どう組み合わせているのか」
「実際に関節を動かすにはどのtopicに何を送ればいいのか」を整理する。
方針・背景の全体像は`docs/計画/アーム動作.md`を参照。このドキュメントは
「Gazebo側の実装が実際どう繋がっているか」に絞った技術解説。

## 1. そもそも何がしたいのか

Go2(4脚)の背中にUnitree D1-T(6軸+グリッパーのアーム)を乗せて、Gazebo上で
既存のNav2パイプラインを崩さずに一緒に動かしたい。ただし今回(Issue #65)で
やったのは「アームの見た目・関節がGazebo上に存在し、topic経由で動かせる状態にする」
までで、「意味のある動き(joint-space 2軸デモ)をさせる」のはIssue #66の仕事。

## 2. データはどこから来ているか

このアーム統合は、由来の異なる3つの要素を組み合わせてできている。混同しやすいので
まず出処を整理する。

| 要素 | 由来 | このプロジェクトでの扱い |
|---|---|---|
| D1アームの形状・質量・関節可動域(URDF) | Unitree公式CDNの`d1_550_description.zip`(`https://oss-global-cdn.unitree.com/static/9b20252a26374d50aa369532657d0143.zip`)。SolidWorks URDF Exporter出力、ROS1/catkin+Gazebo Classic向け | ament_cmake+xacroマクロに移植して`external/go2_ros2_sim_py/d1_550_description`として新規追加(このプロジェクト独自) |
| Go2本体(trunk・脚・センサ)のURDFとGazebo統合設定 | `external/go2_ros2_sim_py`(**自分のfork**、元は`abutalipovvv/go2_ros2_sim_py`。詳細は`外部サブモジュールの使い方.md`) | 既存の`go2_description`にD1アームをfixed jointで追加接続。脚側の設定は無改変 |
| `ros2_control`・`gz_ros2_control`という「関節をtopic経由で動かす」仕組みそのもの | ROS2/Gazebo公式パッケージ(`ros-jazzy-ros2-control`・`ros-jazzy-gz-ros2-control`、simイメージにapt installされている) | 無改変。使い方(どの関節をどのcontrollerにぶら下げるか)だけを設定 |

**注意: 実機用のD1 SDK(`external/unitree_sdk2`、`docker/driver/d1_sdk/`、PR #72)とは別物。**
そちらはUnitree実機とDDSで通信するための実機専用パイプラインで、今回のGazebo統合とは
コードもデータの流れも完全に独立している。両者が繋がるのはIssue #64
(sim/実機で関節目標値を決めるロジックを共通化し、送信先=トランスポート層だけ
差し替える)の時点。今の時点では「Gazebo側」と「実機SDK側」は別々に存在しているだけ、
と考えてよい。

## 3. リポジトリ・ディレクトリ構成

```
Go2_deploy/                              (メインリポジトリ)
├── docker/sim/                          Gazebo(Harmonic)+Nav2コンテナのビルド定義
│                                         (docker composeでexternal/go2_ros2_sim_pyを
│                                          ワークスペースにCOPYしてcolcon build)
├── docs/計画/アーム動作.md               D1アーム全体の方針・調査結果(このドキュメントの親)
└── external/
    ├── go2_ros2_sim_py/                  submodule(自分のfork)
    │   ├── go2_description/              Go2本体(trunk・脚・センサ)のURDF/xacro
    │   │   └── xacro/robot.xacro          ← ここでd1_arm.xacroをincludeして接続
    │   ├── d1_550_description/           ★今回追加。D1アームのROS2/xacro移植版
    │   │   ├── urdf/d1_arm.xacro          xacro:macro本体(関節・リンク定義)
    │   │   ├── urdf/standalone.xacro      単体確認用(RVizでD1だけ表示)
    │   │   ├── meshes/*.STL               ベンダー提供メッシュそのまま
    │   │   └── launch/display.launch.py   単体確認用launch
    │   └── gazebo_sim/launch/            実際の起動launch一式(Nav2込み)
    └── unitree_ros2/                     実機Go2本体との通信SDK(D1アームとは無関係)

docker/driver/d1_sdk/                     (メインリポジトリ、PR #72で追加・未マージ)
                                          実機D1-T用SDKサンプル。今回のGazebo統合とは無関係
```

## 4. xacroの組み込みの流れ

`d1_550_description`は独立したパッケージだが、単体では起動されない。
`go2_description/xacro/robot.xacro`が`xacro:include`で読み込み、`xacro:d1_arm`マクロを
`trunk`リンクへのfixed jointとして1回だけ呼び出すことで、Go2本体のURDFツリーに
接ぎ木される。

```xacro
<!-- robot.xacro -->
<xacro:include filename="$(find d1_550_description)/urdf/d1_arm.xacro"/>
...
<xacro:d1_arm parent="trunk" prefix="d1_" xyz="-0.05 0 0.06" rpy="0 0 0"/>
```

- `prefix="d1_"`: D1側のリンク・関節名(元は`base_link`・`Joint1`等)がGo2本体の
  同名リンクと衝突しないよう、すべて`d1_`を付けてリネームしている
  (`d1_base_link`、`d1_joint1`…`d1_joint6`、`d1_joint_l`/`d1_joint_r`)
- `xyz`/`rpy`: 搭載位置の静的TF。**2026-09-08時点ではまだ実測していない仮値**
  (Gazebo上で目視して見た目が自然になるよう調整しただけ)。実機到着後に
  実測して置き換える(`docs/計画/アーム動作.md` §5、作業計画.md C3)
- 関節の可動域・軸の向き・慣性値はベンダーURDFの値をそのまま使っている。
  **軸の向きは実機未照合**(Caltech AMBER Labの別報告で同系統URDFの2軸が
  実機と食い違っていたと確認済み。詳細は`docs/計画/アーム動作.md` §4-5)

## 5. 起動〜spawnまでのデータフロー

```
docker compose up (docker/sim/compose.yaml)
    │  command: ros2 launch gazebo_sim launch.py
    ▼
gazebo_sim/launch/launch.py            gz sim起動(Harmonic, cafe.world)
    │
    ▼
gazebo_sim/launch/gazebo_multi_nav2_world.launch.py
    │
    ├─ xacro.process_file(robot.xacro) ──▶ robot_description(URDF文字列)
    │                                        (この中にd1_*関節・リンクが含まれる)
    ├─ robot_state_publisher ──publish──▶ /robot1/robot_description(topic)
    ├─ ros_gz_sim create -topic /robot1/robot_description
    │      └─▶ gzが受け取ってモデルをspawn。<ros2_control>タグを見て
    │          gz_ros2_control::GazeboSimROS2ControlPlugin をロード
    ├─ controller_manager spawner ×3
    │      ├─ joint_state_broadcaster        (脚+アーム全関節の状態を配信)
    │      ├─ joint_group_controller         (脚12関節。既存の歩行ロジック用)
    │      └─ d1_arm_controller              (D1アーム8関節。★今回追加)
    └─ Nav2 bringup / RViz(既存のまま、無変更)
```

脚用の`joint_group_controller`とアーム用の`d1_arm_controller`は**意図的に別系統**にした。
理由は、歩行ロジック(`quadropted_controller`)が`joint_group_controller`へ脚12関節分の
配列を送り続けているため、もしアーム関節を同じcontrollerに混ぜると歩行ノード側の
配列長・関節順序を変更する必要が生じ、既存の歩行が壊れるリスクがあったため。

## 6. 実際に関節を動かす(検証済みのI/O)

| 項目 | 値 |
|---|---|
| コマンドtopic | `/robot1/d1_arm_controller/commands` |
| 型 | `std_msgs/msg/Float64MultiArray` |
| 配列の順番 | `d1_joint1, d1_joint2, d1_joint3, d1_joint4, d1_joint5, d1_joint6, d1_joint_l, d1_joint_r`(ラジアン。グリッパー2軸はメートル、`ros_control.yaml`の`joints:`順) |
| 状態を見るtopic | `/robot1/joint_states`(`sensor_msgs/msg/JointState`。脚とアーム全関節が同じ配列に混在) |

動作確認コマンド(実際にこのセッションで実行し、Gazebo上でアームが動くことを確認済み):

```bash
docker exec go2-sim bash -c "source /opt/ros/jazzy/setup.bash && \
  source /root/ws/install/setup.bash && \
  ros2 topic pub -t 1 /robot1/d1_arm_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.5, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'"
```

## 7. 概念⇔実装 対応表

| 概念 | 実際のパッケージ・ファイル |
|---|---|
| D1アームの形状・質量・関節可動域データ | `d1_550_description/urdf/d1_arm.xacro`(Unitree公式URDFの移植) |
| Go2本体へのマウント(静的TF、仮値) | `go2_description/xacro/robot.xacro`の`xacro:d1_arm`呼び出し |
| 関節をtopicから動かす仕組み | `go2_description/xacro/gazebo.xacro`の`D1ArmGazeboSystem`(`ros2_control`) |
| どの関節をどんなcontrollerにまとめるか | `go2_description/config/ros_control.yaml`の`d1_arm_controller` |
| controllerの起動 | `gazebo_sim/launch/gazebo_multi_nav2_world.launch.py`の`d1_arm_controller`スポナー |
| 実機との対応(将来、#64) | `docker/driver/d1_sdk/`(PR #72、未マージ、今は無関係) |

## 8. 今わかっている限界・注意点

- **搭載位置(静的TF)は実測ではなく仮値**。目視で「それっぽい」位置に置いただけ
  (§4参照)。実機のGo2背面にD1-Tを実際に固定する位置が決まるまでは、この数値に
  意味はない
- **関節の回転軸は実機未照合**。ベンダーURDFをそのまま使っており、他ラボの報告
  (Caltech AMBER Lab)では同系統URDFで2軸が実機と食い違っていたとされる
- **PID/ゲイン未調整で、関節が重力に負けて可動域の端まで振れることがある**:
  §6の動作確認で`d1_joint2`に`0.3`を指令したところ、実際には重力に引かれて
  可動域上限の`1.57`(rad)まで振れて張り付く挙動が確認された。`ros2_control`の
  `<ros2_control>`ブロックにはゲイン(`p`/`i`/`d`)を明示していない
  (脚側も同様に未指定で、既存の歩行制御が動いているのは歩容ノードが継続的に
  指令を送り続けているため)。**Issue #66でjoint-space 2軸デモを作る前に、
  D1アーム用のゲイン(またはPID)を調整する必要がある**
- **脚用`joint_group_controller`とアーム用`d1_arm_controller`は別系統**。歩行ロジックを
  壊さないための意図的な分離であり、今後もこの2つを混ぜないこと

## 9. 次にやること(Issue #66)

- D1アーム用のPID/ゲイン調整(§8、重力による可動域端への張り付きを解消)
- 「J1(ベース旋回)+もう1軸を直接joint-space角度指定で動かし、残り4軸は中立姿勢固定」
  というsim/実機共通ロジック(`docs/計画/アーム動作.md` §3-1)を、`/robot1/d1_arm_controller/commands`
  へpublishするノードとして実装
- 静的TF・関節軸の実機照合は実機到着後(Issue #65の残タスク、§8前半2点)
