cmake_minimum_required(VERSION 3.8)
project(curie_arm_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_library(antidrift_component SHARED src/servo_antidrift.cpp)
ament_target_dependencies(
  antidrift_component PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
)
target_include_directories(antidrift_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(
    antidrift_component
    PLUGIN "ServoAntiDrifter"
    EXECUTABLE servo_antidrift
)

ament_export_targets(export_antidrift_component)
install(TARGETS antidrift_component
        EXPORT export_antidrift_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_executable(move_joints movement_plans/move_joints.cpp)
add_executable(move_zig_zag movement_plans/move_zig_zag.cpp)
ament_target_dependencies(
  move_joints
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)
ament_target_dependencies(
  move_zig_zag
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  move_joints
  move_zig_zag
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
