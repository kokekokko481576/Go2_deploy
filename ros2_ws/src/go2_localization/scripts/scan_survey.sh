#!/usr/bin/env bash
# 部屋の複数地点でチン(顎)LiDARの有効反射数を確認する簡易サーベイ(#54切り分け)。
set -eu

POINTS=(
  "1.0 0.0 テストコース1(1,0)"
  "3.0 0.0 テストコース2(3,0)"
  "2.0 -1.0 GATE1範囲隅(2,-1)"
  "1.0 1.0 GATE1範囲隅(1,1)"
  "0.6 -1.3 table1近く(0.6,-1.3)"
  "4.0 0.0 東壁近く(4.0,0)"
  "0.0 0.0 原点/スポーン地点(0,0)"
)

for p in "${POINTS[@]}"; do
  x=$(echo "$p" | cut -d' ' -f1)
  y=$(echo "$p" | cut -d' ' -f2)
  label=$(echo "$p" | cut -d' ' -f3-)

  docker exec go2-sim bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null && gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req \"name: 'robot1_my_bot', position: {x: ${x}, y: ${y}, z: 0.46}, orientation: {x: 0, y: 0, z: 0, w: 1}\"" > /dev/null 2>&1
  sleep 1.5

  docker exec arbeit-ros2 bash -c "source /opt/ros/humble/setup.bash && source /home/ros/ros2_ws/install/setup.bash && timeout 4 ros2 topic echo /go2_localization/chin_lidar_scan --once 2>/dev/null" > /tmp/scan_survey_tmp.yaml 2>/dev/null || true

  python3 - "$label" "$x" "$y" <<'PYEOF'
import sys, yaml, math
label, x, y = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    docs = list(yaml.safe_load_all(open('/tmp/scan_survey_tmp.yaml')))
    doc = docs[0]
    ranges = doc['ranges']
    finite = [r for r in ranges if isinstance(r, float) and math.isfinite(r)]
    within_2m = [r for r in finite if r <= 2.0]
    print(f"{label:28s} ({x},{y}): 有効={len(finite):3d}/{len(ranges)}  2m以内={len(within_2m):3d}  "
          f"最短={min(finite) if finite else float('nan'):.2f}m")
except Exception as e:
    print(f"{label:28s} ({x},{y}): 読み取り失敗 ({e})")
PYEOF
done
