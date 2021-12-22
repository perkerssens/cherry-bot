cmake_minimum_required(VERSION 3.8)
project(cherry_bot_moveit_config)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()