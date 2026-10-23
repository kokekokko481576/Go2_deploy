#!/bin/bash
# Gazebo（go2-sim コンテナ）に、検査部材（逆T字）と録画用の固定カメラを置く（2026-09-29、動画撮影用）。
# 配置は実機に合わせる: タグ(-4.96, 1.5、法線+X)の手前0.65mで止まった機体（向き-X）の右側(+Y)に、
# 右足の列から20cm空けて部材を置く。機体の中心線は部材の手前の縁から0.342m。
# 部材: ベース板 700x300x12mm（左端=機体の前の股関節の位置）、縦板 410x10x100mm（手前の縁から140mm）
set -u
C=go2-sim
R() { docker exec "$C" bash -c ". /opt/ros/jazzy/setup.bash && source \$WORKSPACE_DIR/install/setup.bash && $1"; }
# 世界座標: 機体の止まる位置 x=-4.31, y=1.5。前の股関節 x=-4.503。部材の手前の縁 y=1.842
# MEMBER_DX / MEMBER_DY[m] で部材をずらせる（2026-10-07、撮影位置の補正の動画用。+Y=機体から遠ざかる）
MX=$(python3 -c "print(round(-4.153 + ${MEMBER_DX:-0}, 4))"); MY=$(python3 -c "print(round(1.992 + ${MEMBER_DY:-0}, 4))")
MEMBER_SDF='<sdf version="1.9"><model name="member"><static>true</static><pose>'"$MX $MY"' 0 0 0 0</pose>
<link name="base"><pose>0 0 0.006 0 0 0</pose>
 <visual name="v"><geometry><box><size>0.70 0.30 0.012</size></box></geometry>
  <material><ambient>0.35 0.30 0.25 1</ambient><diffuse>0.45 0.38 0.30 1</diffuse></material></visual>
 <collision name="c"><geometry><box><size>0.70 0.30 0.012</size></box></geometry></collision></link>
<link name="web"><pose>-0.145 -0.01 0.062 0 0 0</pose>
 <visual name="v"><geometry><box><size>0.41 0.010 0.100</size></box></geometry>
  <material><ambient>0.30 0.30 0.32 1</ambient><diffuse>0.38 0.38 0.40 1</diffuse></material></visual>
 <collision name="c"><geometry><box><size>0.41 0.010 0.100</size></box></geometry></collision></link>
<link name="bead"><pose>-0.145 -0.018 0.014 0 0 0</pose>
 <visual name="v"><geometry><box><size>0.41 0.008 0.006</size></box></geometry>
  <material><ambient>0.85 0.75 0.40 1</ambient><diffuse>0.95 0.85 0.45 1</diffuse></material></visual></link>
</model></sdf>'
# 録画用の固定カメラ: 出発位置の斜め後ろ上から、機体・部材・奥の壁のマーカーを見下ろす（候補を3つ比べて選んだ）
CAM_SDF='<sdf version="1.9"><model name="overview_cam"><static>true</static>
<pose>-3.2 0.6 1.7 0 0.8374 2.3086</pose>
<link name="l"><sensor name="overview" type="camera"><update_rate>15</update_rate>
 <camera><horizontal_fov>1.0</horizontal_fov><image><width>1280</width><height>720</height></image>
 <clip><near>0.05</near><far>30</far></clip></camera><topic>/overview_cam</topic></sensor></link>
</model></sdf>'
for pair in "member:$MEMBER_SDF" "overview_cam:$CAM_SDF"; do
  name=${pair%%:*}; sdf=${pair#*:}
  R "gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 --req 'name: \"$name\", type: MODEL'" >/dev/null 2>&1
  esc=$(printf '%s' "$sdf" | sed 's/"/\\"/g' | tr -d '\n')
  R "gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req 'sdf: \"$esc\"'"
done
# カメラを ROS へ（録画ノードが購読する）
docker exec "$C" bash -c 'ps -ef | grep "[o]verview_cam@" | awk "{print \$2}" | xargs -r kill' 2>/dev/null
docker exec -d "$C" bash -c ". /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /overview_cam@sensor_msgs/msg/Image[gz.msgs.Image > /tmp/overview_bridge.log 2>&1"
echo "[spawn_scene] 部材と録画用カメラを置きました"
