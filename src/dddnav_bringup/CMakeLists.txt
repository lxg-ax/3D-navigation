cmake_minimum_required(VERSION 3.5)
project(dddnav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  map
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
