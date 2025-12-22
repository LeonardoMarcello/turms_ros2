cmake_minimum_required(VERSION 3.10)
project(ros_rpicam)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(${LIBCAMERA_INCLUDE_DIRS})
link_directories(${LIBCAMERA_LIBRARY_DIRS})

include_directories(
  include
  ${LIBCAMERA_INCLUDE_DIRS}
)

add_executable(rpi_cam_node
  src/main.cpp
  src/rpi_cam_node.cpp
)

ament_target_dependencies(rpi_cam_node
  rclcpp
  sensor_msgs
  camera_info_manager
  cv_bridge
  OpenCV
)

target_link_libraries(rpi_cam_node
  ${LIBCAMERA_LIBRARIES}
)

install(TARGETS rpi_cam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
