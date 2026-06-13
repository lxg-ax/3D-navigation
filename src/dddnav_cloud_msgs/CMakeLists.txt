cmake_minimum_required(VERSION 3.8)
project(dddnav_cloud_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED PATHS /opt/ros/humble NO_DEFAULT_PATH)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
