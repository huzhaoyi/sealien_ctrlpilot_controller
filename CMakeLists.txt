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
# find_package(tf2 REQUIRED)
# find_package(tf2_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include  
  ${yaml-cpp_INCLUDE_DIRS}  
  SYSTEM  
  )


add_executable(controller
              src/controller_node.cpp
              src/controller_core.cpp)
target_link_libraries(controller libyaml-cpp.a)
ament_target_dependencies(controller rclcpp std_msgs geometry_msgs sealien_ctrlpilot_msgmanagement)


install(TARGETS 
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

