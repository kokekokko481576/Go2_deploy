#!/usr/bin/env bash
# 自作AMCLのパーティクルを地図全体に撒き直す(大域再自己位置推定、Issue #27)。
# 「ロボットが今どこにいるか自分でも分からない」ときの復旧手段その2。
# 撒き直しただけでは収束しない。実行後にteleop等でロボットを動かし回すと、
# 観測と合う場所にパーティクルが寄っていく(適応リカバリも有効化済みなので、
# 動かしているうちに正しい位置へ収束していく)。
#
# 使い方(devコンテナ内、localization起動中に):
#   ./relocalize_global.sh
#   → その後 teleop で動かしながら ros2 topic echo /go2_localization/amcl_pose で収束を見る
set -eu

echo "call /reinitialize_global_localization (パーティクルを地図全体へ撒き直し)"
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
echo "完了。teleopでロボットを動かして収束させてください(静止したままでは収束しない)"
