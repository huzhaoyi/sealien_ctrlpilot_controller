cmake_minimum_required(VERSION 3.8)
project(sealien_ctrlpilot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sealien_ctrlpilot_msgmanagement REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(
  include  
  ${yaml-cpp_INCLUDE_DIRS}  
  SYSTEM  
  )


add_executable(controller
              src/controller_node.cpp
              src/controller_core.cpp
              src/ctr_mode.cpp
              src/action_function.cpp)
target_link_libraries(controller libyaml-cpp.a)
ament_target_dependencies(controller 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sealien_ctrlpilot_msgmanagement
  tf2_geometry_msgs
  tf2_msgs
  tf2
  GeographicLib
  nav_msgs
  sensor_msgs)

add_executable(msg_adapter_node
              src/msgs_adapter_node.cpp)
target_link_libraries(msg_adapter_node libyaml-cpp.a)
ament_target_dependencies(msg_adapter_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sealien_ctrlpilot_msgmanagement
  tf2_geometry_msgs
  tf2_msgs
  tf2
  GeographicLib
  nav_msgs
  sensor_msgs)


install(TARGETS 
  controller
  msg_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY 
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ctrl_bringup.sh
  scripts/auv_stack.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  systemd/ctrl_bringup.service
  DESTINATION share/${PROJECT_NAME}/systemd
)

ament_package()

