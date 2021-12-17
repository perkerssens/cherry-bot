cmake_minimum_required(VERSION 3.8)
project(cherry_bot_description)

find_package(ament_cmake REQUIRED)

# Install urdf files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()