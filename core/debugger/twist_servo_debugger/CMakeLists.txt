cmake_minimum_required(VERSION 3.5)
project(twist_servo_debugger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_message REQUIRED)
find_package(realtime_tools REQUIRED)

add_executable(twist_servo_debugger_node src/twist_servo_debugger_node.cpp)

target_include_directories(twist_servo_debugger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  twist_servo_debugger_node
  rclcpp
  geometry_msgs
  rm_message
  realtime_tools
)

install(TARGETS
  twist_servo_debugger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
