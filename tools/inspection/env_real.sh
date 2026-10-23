# 実機作業の共通環境。**各端末で最初に必ず source すること。**
#   source ~/marker_detection/tools/env_real.sh
#
# CYCLONEDDS_URI を入れ忘れると、機体のトピックが見えない/見えても届かない。
# ノードごとに設定が食い違うと「起動しているのに何も来ない」という形で出る。
# ROSのsetup.bashは未定義変数を参照するので set -u を掛けたまま source しない。
set +u
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/install/setup.bash"
source "$HOME/marker_detection/ros2_ws/install/setup.bash"
set -u 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
<NetworkInterface name="enp2s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
echo "[env_real] 実機用の環境を設定しました（enp2s0 / CycloneDDS）"
