cmake_minimum_required(VERSION 3.5)
project(dddnav_utils)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/livox_pc2_to_liosam.py
  scripts/liosam_to_posegraph.py
  scripts/fastlio_to_posegraph.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
