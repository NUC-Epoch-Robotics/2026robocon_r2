cmake_minimum_required(VERSION 3.8)
project(r2_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(r2_interfaces REQUIRED)

add_executable(r2_decision_node
  src/r2_decision_node.cpp
)

ament_target_dependencies(r2_decision_node
  rclcpp
  rclcpp_action
  nav2_msgs
  r2_interfaces
)

install(TARGETS
  r2_decision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
