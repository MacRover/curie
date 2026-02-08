cmake_minimum_required(VERSION 3.8)
project(curie_hw_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Creating and exporting hardware library to other packages
add_library(
  hw_lib SHARED 
  lib/spark_mmrt/src/device/SparkMax.cpp
  lib/spark_mmrt/src/frames/SparkFrames.cpp
  lib/spark_mmrt/src/can/SocketCanTransport.cpp

  interfaces/spark_drive_hardware.cpp
)
target_include_directories(hw_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/spark_mmrt/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interfaces>
  $<INSTALL_INTERFACE:include>)
ament_export_libraries(hw_lib)
ament_export_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/spark_mmrt/include
  ${CMAKE_CURRENT_SOURCE_DIR}/interfaces)

add_executable(test_sparkmax src/test_sparkmax.cpp)
ament_target_dependencies(test_sparkmax rclcpp std_msgs)
target_include_directories(test_sparkmax PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/spark_mmrt/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(test_sparkmax hw_lib)

install(TARGETS test_sparkmax
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS hw_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
