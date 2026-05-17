cmake_minimum_required(VERSION 3.5)
project(dddnav_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++ port of the Livox→LIO-SAM bridge. Avoids Python GIL on the 10 Hz
# pointcloud path; per-point `time` is no longer affected by interpreter
# stalls. The Python script is kept under scripts/ as a fallback.
add_executable(livox_pc2_to_liosam src/livox_pc2_to_liosam.cpp)
ament_target_dependencies(livox_pc2_to_liosam rclcpp sensor_msgs)

install(TARGETS
  livox_pc2_to_liosam
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/livox_pc2_to_liosam.py
  scripts/liosam_to_posegraph.py
  scripts/fastlio_to_posegraph.py
  scripts/slam_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
