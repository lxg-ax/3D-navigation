cmake_minimum_required(VERSION 3.5)
project(dddnav_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(Eigen3 REQUIRED)
find_package(dddnav_std_descriptor REQUIRED)

add_executable(livox_pc2_to_liosam src/livox_pc2_to_liosam.cpp)
ament_target_dependencies(livox_pc2_to_liosam rclcpp sensor_msgs)

# STD-based global localization bootstrap + watchdog (replaces the
# operator-supplied /initial_3d_pose seed). Reads <map>/lio_sam/std_db.bin
# dumped by mapOptimization::saveMap.
add_executable(std_global_init src/std_global_init.cpp)
target_include_directories(std_global_init PRIVATE
  ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(std_global_init
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs pcl_conversions
  dddnav_std_descriptor)
target_link_libraries(std_global_init ${PCL_LIBRARIES})

install(TARGETS
  livox_pc2_to_liosam
  std_global_init
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/liosam_to_posegraph.py
  scripts/fastlio_to_posegraph.py
  scripts/slam_health_monitor.py
  scripts/save_map_on_exit.py
  scripts/nav_perf_monitor.py
  scripts/dddnav_preflight.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
