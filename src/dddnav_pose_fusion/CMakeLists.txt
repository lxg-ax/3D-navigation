cmake_minimum_required(VERSION 3.5)
project(dddnav_pose_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

set(deps
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_fusion_node
  src/pose_fusion_node.cpp
  src/eskf_se3.cpp
)
ament_target_dependencies(pose_fusion_node ${deps})

install(TARGETS pose_fusion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_eskf_se3 test/test_eskf_se3.cpp src/eskf_se3.cpp)
  if(TARGET test_eskf_se3)
    target_include_directories(test_eskf_se3 PRIVATE
      include ${EIGEN3_INCLUDE_DIRS})
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${deps})
ament_package()
