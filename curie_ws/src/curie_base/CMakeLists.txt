cmake_minimum_required(VERSION 3.8)
project(curie_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(base_component SHARED src/base.cpp)
ament_target_dependencies(
  base_component PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
)
target_include_directories(base_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(
    base_component
    PLUGIN "base::Basestation"
    EXECUTABLE base_driver
)

ament_export_targets(export_base_component)
install(TARGETS base_component
        EXPORT export_base_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_package()
