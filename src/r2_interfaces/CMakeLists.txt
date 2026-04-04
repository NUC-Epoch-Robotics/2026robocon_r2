cmake_minimum_required(VERSION 3.8)
project(r2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalPose.msg"
  "msg/GoalReached.msg"
  "msg/ArmCommand.msg"
  "msg/ArmDone.msg"
  "msg/ArmAck.msg"
)

ament_package()
