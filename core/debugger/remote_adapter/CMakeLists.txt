cmake_minimum_required(VERSION 3.5)
project(remote_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_message REQUIRED)

add_executable(remote_adapter_node src/remote_adapter_node.cpp)

target_include_directories(remote_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  remote_adapter_node
  rclcpp
  std_msgs
  rm_message
)

install(TARGETS
  remote_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install params and launch files
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
