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
find_package(std_srvs REQUIRED)

add_library(base_main_component SHARED src/base.cpp)
ament_target_dependencies(
  base_main_component PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_srvs
)
target_include_directories(base_main_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(
    base_main_component
    PLUGIN "base::Basestation"
    EXECUTABLE base_main
)

ament_export_targets(export_base_main_component)
install(TARGETS base_main_component
        EXPORT export_base_main_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install launch and config files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
