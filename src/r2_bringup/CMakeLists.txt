cmake_minimum_required(VERSION 3.8)
project(r2_bringup)

find_package(ament_cmake REQUIRED)

# 安装 launch 文件
install(
  DIRECTORY launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
