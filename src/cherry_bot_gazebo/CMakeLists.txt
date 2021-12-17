cmake_minimum_required(VERSION 3.8)
project(cherry_bot_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY
  launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()