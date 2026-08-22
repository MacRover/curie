cmake_minimum_required(VERSION 3.22)
project(moveit_testing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(keyboard_interfaces REQUIRED)

add_executable(moveit_testing moveit_testing/moveit_testing.cpp)
ament_target_dependencies(moveit_testing 
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
  keyboard_interfaces
)

install(TARGETS moveit_testing 
  DESTINATION lib/${PROJECT_NAME}
) 
ament_package()