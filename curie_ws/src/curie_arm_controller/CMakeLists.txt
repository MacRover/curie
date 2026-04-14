cmake_minimum_required(VERSION 3.8)
project(curie_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(curie_hw_control REQUIRED)

add_library(
  curie_arm_controller
  SHARED
  hardware/arm_controller.cpp
)

target_compile_features(curie_arm_controller PUBLIC cxx_std_17)
target_include_directories(curie_arm_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/curie_arm_controller>
)
ament_target_dependencies(
  curie_arm_controller PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  curie_hw_control
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface curie_arm_controller.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/curie_arm_controller
)

install(TARGETS curie_arm_controller
  EXPORT curie_arm_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
