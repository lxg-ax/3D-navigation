cmake_minimum_required(VERSION 3.10)
project(dddnav_std_descriptor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wno-unused-parameter -Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)

# OpenMP knob expected by upstream STDesc.cpp (#ifdef MP_EN).
add_definitions(-DMP_EN -DMP_PROC_NUM=4)

add_library(dddnav_std_descriptor SHARED
  src/STDesc.cpp
)
target_include_directories(dddnav_std_descriptor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(dddnav_std_descriptor
  ${PCL_LIBRARIES}
  Ceres::ceres
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(dddnav_std_descriptor rclcpp)

install(TARGETS dddnav_std_descriptor
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(dddnav_std_descriptor)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 PCL Ceres OpenMP)

ament_package()
