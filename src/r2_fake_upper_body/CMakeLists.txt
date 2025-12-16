cmake_minimum_required(VERSION 3.8)
project(r2_fake_upper_body)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(r2_interfaces REQUIRED)

add_executable(fake_upper_body_node
  src/fake_upper_body_node.cpp
)

ament_target_dependencies(fake_upper_body_node
  rclcpp
  r2_interfaces
)

install(TARGETS
  fake_upper_body_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
