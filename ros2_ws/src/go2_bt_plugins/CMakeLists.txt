cmake_minimum_required(VERSION 3.8)
project(go2_bt_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(go2_is_collision_detected_condition_bt_node SHARED
  src/is_collision_detected_condition.cpp)
ament_target_dependencies(go2_is_collision_detected_condition_bt_node
  behaviortree_cpp_v3
  nav2_behavior_tree
  rclcpp
  std_msgs)

install(TARGETS go2_is_collision_detected_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(go2_is_collision_detected_condition_bt_node)
ament_export_dependencies(behaviortree_cpp_v3 nav2_behavior_tree rclcpp std_msgs)

ament_package()
