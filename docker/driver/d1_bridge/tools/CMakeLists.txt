# d1_bridge の検査用ツール。**colcon ではなく素の cmake でビルドする**
# (unitree_sdk2 にリンクするため。ROS2 側とは別プロセスで動かすことで
#  CycloneDDS の ABI 衝突を避ける)。メッセージ定義は d1_sdk のものを流用する。
project(d1_bridge_tools)
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2 /root/d1_sdk/src)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
add_executable(dds_probe dds_probe.cpp /root/d1_sdk/src/msg/ArmString_.cpp)
