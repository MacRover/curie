cmake_minimum_required(VERSION 3.8)
project(curie_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(curie_hw_control REQUIRED)

add_library(
  curie_drive_controller
  SHARED
  hardware/drive_controller.cpp
)

target_compile_features(curie_drive_controller PUBLIC cxx_std_17)
target_include_directories(curie_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/curie_drive_controller>
)
ament_target_dependencies(
  curie_drive_controller PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  curie_hw_control
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface curie_drive_controller.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/curie_drive_controller
)

install(TARGETS curie_drive_controller
  EXPORT curie_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
