cmake_minimum_required(VERSION 3.16)#3.16
project(turms_ros2control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware REQUIRED)


## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/turms_system.cpp
  )
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_manager
  #serial
  rclcpp
  pluginlib
  hardware
)
target_link_libraries(
  ${PROJECT_NAME}
  serial
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface turms_ros2control.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  #serial
  rclcpp
  pluginlib
)
ament_package()
