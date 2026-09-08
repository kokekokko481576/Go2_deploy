#!/bin/bash
# Gazebo(Go2_deploy の simコンテナ)で、マーカー接近＋真横旋回を動かす。
#
# 前提: go2-sim コンテナが起動していること（compose.yaml が ros2_ws を /ros2_ws へ
#       読み取り専用でマウントする）。**計測するなら Nav2 は切ること**:
#         cd docker/sim && SIM_ENABLE_NAV2=false docker compose up -d
#       既定では upstream の Nav2 が /robot1/cmd_vel に publisher を6個持ち、
#       こちらのノードが止めたあとに機体を動かして計測を汚す。
#
# 使い方:
#   docker/sim/tools/sim_up.sh                 # 起動して待機（enableは別途）
#   docker/sim/tools/sim_up.sh --place         # タグの手前2mへ瞬間移動させてから起動
#   docker/sim/tools/sim_up.sh --place --go    # 置いて、起動して、開始まで行う
#
# **接近制御はコンテナの中(Jazzy)で動かす。** ホスト(Humble)から動かそうとすると
# /tf の受信が `invalid data size, at serdata.cpp` で失敗する（ディストロ跨ぎの
# シリアライズ非互換）。標準型なら通るという前提は、少なくともTFでは成り立たない。
set -u
C=go2-sim
R() { docker exec "$C" bash -c ". /opt/ros/jazzy/setup.bash && $1"; }
RD() { docker exec -d "$C" bash -c ". /opt/ros/jazzy/setup.bash && $1"; }

# --- タグ検出のパラメータ（**実測で決めた値。理由は README を読むこと**）---
# size: モデル同梱の設定は 0.08 だが、真値と突き合わせて測るとスケール誤差 +3.8%。
#       decimate=1.0 の条件で測り直して 0.078434 に合わせた（誤差 +0.03%、固定オフセット +10mm）。
# decimate: 既定の 2.0 では 8cm のタグを 2.0m までしか検出できない。1.0 で 2.4m まで伸びる。
TAG_SIZE=0.078434
DECIMATE=1.0
# --- ヨー角の出所 ---
# **simでは IMU を使う。** 真横への90度旋回で機体は約14cm引きずられ、その分だけ
# マーカーの方位が約10度変わる。制御則は位置も渡せば推測航法で補正できるが、
# **simの `/robot1/odometry/filtered` はこの移動を観測できず(116mmに対し5mm)、
# しかもヨーが180度飛ぶことがある**。位置を渡すと3回中2回が逆向き・無回転に
# なった（2026-09-08実測）。IMUのヨーは真値と0.05度で一致し安定しているので、
# **約10度の既知のずれを受け入れて、そちらを使う**（最終合わせはアーム搭載カメラの担当）。
# 実機では /sportmodestate の位置が引きずりを捉えるかもしれない（未検証）。
# --- 視野の予算（**実機と違う。simのカメラは水平±31度しかない**）---
# 制御則の既定 25/33度 は実機の±46度前提。simでそのまま使うと直進中にタグが
# 画面から切れて見失う。実測に合わせて絞る。
FOV_BUDGET=12.0
DRIVE_LIMIT=18.0

if [[ " $* " == *" --place "* ]]; then
    # タグ(-4.96, 1.5、法線は+X向き)の手前約2m、法線から約5度の位置へ置く
    R 'source "$WORKSPACE_DIR/install/setup.bash" && gz service -s /world/default/set_pose \
        --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
        --req "name: \"robot1_my_bot\", position: {x: -3.0, y: 1.68, z: 0.45}, \
               orientation: {x: 0, y: 0, z: 1, w: 0}"' >/dev/null
    # 置いた直後に前の指令が残っていると歩き出す。**simには cmd_vel の
    # ウォッチドッグが無い**ので、明示的にゼロを送る
    R 'ros2 topic pub -t 3 /robot1/cmd_vel geometry_msgs/msg/Twist "{}"' >/dev/null 2>&1
    echo "[sim_up] ロボットをタグの手前2m・法線から5度の位置に置きました"
fi

# 既存のプロセスを落とす。**pkill -f は自分自身にマッチして取り逃す**ので PID で殺す
docker exec "$C" bash -c 'ps -ef | grep -E "[a]priltag_node|[s]im_tag_bridge|[m]arker_approach.approach_node" | awk "{print \$2}" | xargs -r kill -9' 2>/dev/null
sleep 2

RD "ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/robot1/color/image_raw -r camera_info:=/robot1/color/camera_info \
    -p family:=36h11 -p size:=$TAG_SIZE -p detector.decimate:=$DECIMATE \
    -p use_sim_time:=true > /tmp/apriltag.log 2>&1"
RD "python3 /sim_tools/sim_tag_bridge.py > /tmp/bridge.log 2>&1"
RD "export PYTHONPATH=/ros2_ws/src/marker_approach:\$PYTHONPATH && \
    python3 -m marker_approach.approach_node --ros-args \
    -r cmd_vel_raw:=/robot1/cmd_vel \
    -p camera_x:=0.33 -p camera_y:=0.0 -p camera_z:=0.0057 \
    -p standoff:=0.65 -p dry_run:=false -p pos_tolerance:=0.09 \
    -p final_heading:=right -p side_turn_angle_deg:=81.0 -p yaw_source:=imu -p yaw_topic:=/robot1/imu_plugin/out \
    -p fov_budget_deg:=$FOV_BUDGET -p drive_bearing_limit_deg:=$DRIVE_LIMIT \
    -p max_runtime:=120.0 > /tmp/approach.log 2>&1"
sleep 5
echo "[sim_up] 起動しました。ログ: docker exec $C tail -f /tmp/approach.log"
echo "[sim_up] 開始:  docker exec $C bash -c '. /opt/ros/jazzy/setup.bash && \\"
echo "                 ros2 topic pub --once /marker_approach_node/enable std_msgs/msg/Bool \"{data: true}\"'"

if [[ " $* " == *" --go "* ]]; then
    R 'ros2 topic pub --once /marker_approach_node/enable std_msgs/msg/Bool "{data: true}"' >/dev/null 2>&1
    echo "[sim_up] 開始しました"
fi
