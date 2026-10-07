project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

# include パスは unitree_sdk2 の CMake 設定(unitree_sdk2Targets.cmake)から受け取る。
# 以前はバージョン入りのパス(/usr/local/include/iceoryx/v2.0.2)を直書きしていたが、
# 同梱の CycloneDDS は iceoryx 無しでビルドされていてそのディレクトリは存在しない(不要だった)。
find_package(unitree_sdk2 REQUIRED)
link_libraries(unitree_sdk2 rt)

add_executable(joint_angle_control src/joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(multiple_joint_angle_control src/multiple_joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(joint_enable_control src/joint_enable_control.cpp src/msg/ArmString_.cpp)
add_executable(arm_zero_control src/arm_zero_control.cpp src/msg/ArmString_.cpp)
add_executable(get_arm_joint_angle src/get_arm_joint_angle.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
add_executable(restore_initial_pose src/restore_initial_pose.cpp src/msg/ArmString_.cpp)
