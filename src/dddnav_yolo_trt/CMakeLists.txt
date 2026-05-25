cmake_minimum_required(VERSION 3.5)
project(dddnav_yolo_trt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  builtin_interfaces
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

#Check yolo build flag
if(NOT TRT_ENABLED)
    set(TRT_ENABLED OFF CACHE BOOL "Enable or Disable TensorRT")
endif()

if(TRT_ENABLED)

  find_package(CUDAToolkit REQUIRED)
  message(STATUS "TensorRT is enabled. Compiling TensorRT and Yolo")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake")
  add_subdirectory(src/yolo/yolo_trt/tensorrt-cpp-api)

  add_library(dddnav_yolo_trt SHARED src/yolo/yolo_trt/yolov8.cpp)
  target_link_libraries(dddnav_yolo_trt PUBLIC tensorrt_cpp_api ${OpenCV_LIBRARIES} CUDA::cudart)
  target_include_directories(dddnav_yolo_trt PUBLIC src/yolo/yolo_trt/tensorrt-cpp-api/src ${CUDAToolkit_INCLUDE_DIRS})

  install(
    DIRECTORY include/ src/yolo/yolo_trt/tensorrt-cpp-api/src/
    DESTINATION include/
  )

  install(TARGETS
    dddnav_yolo_trt
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_include_directories(include)
  ament_export_libraries(dddnav_yolo_trt)
  # --- Everything CUDA finish here ---

else()
    message(STATUS "TensorRT is disabled.")
endif()

ament_package()
