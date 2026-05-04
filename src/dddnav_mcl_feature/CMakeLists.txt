cmake_minimum_required(VERSION 3.5)
project(dddnav_mcl_feature)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cloud_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
  tf2_ros tf2 tf2_geometry_msgs tf2_sensor_msgs tf2_eigen
  pcl_conversions cloud_msgs cv_bridge image_geometry
)

include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# imageProjection library (no TRT/YOLO for MCL mode)
add_library(mcl_image_projection SHARED src/imageProjection.cpp)
target_link_libraries(mcl_image_projection ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(mcl_image_projection ${dependencies})

# featureAssociation library
add_library(mcl_feature_association SHARED src/featureAssociation.cpp)
target_link_libraries(mcl_feature_association ${PCL_LIBRARIES})
ament_target_dependencies(mcl_feature_association ${dependencies})

# mcl_feature executable
add_executable(mcl_feature src/mcl_feature_node.cpp)
target_link_libraries(mcl_feature mcl_image_projection mcl_feature_association)
ament_target_dependencies(mcl_feature ${dependencies})

install(TARGETS mcl_image_projection mcl_feature_association
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS mcl_feature RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(mcl_image_projection mcl_feature_association)
ament_export_dependencies(${dependencies})
ament_package()
