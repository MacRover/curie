cmake_minimum_required(VERSION 3.8)
project(curie_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

SET(robot_descriptions 
    mock_diffbot
    arm
)

foreach(robot_description ${robot_descriptions})
  foreach(dir rviz urdf meshes launch)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${robot_description}/${dir}")
      install(DIRECTORY ${robot_description}/${dir} 
              DESTINATION share/${PROJECT_NAME}/${robot_description})
    endif()
  endforeach(dir)
endforeach(robot_description)

ament_package()
