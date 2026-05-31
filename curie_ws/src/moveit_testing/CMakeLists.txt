cmake_minimum_required(VERSION 3.22)
project(moveit_testing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_testing moveit_testing/moveit_testing.cpp)
ament_target_dependencies(moveit_testing 
    rclcpp
    moveit_ros_planning_interface
)

install(TARGETS moveit_testing 
    DESTINATION lib/${PROJECT_NAME}
) 

ament_package()

