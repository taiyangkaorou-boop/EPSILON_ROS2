cmake_minimum_required(VERSION 3.5)
project(playgrounds)

# 数据包本身不编译 C++，仍沿用工作空间统一的 C++17 配置。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 只需要 ament_cmake 完成资源安装和包索引注册。
find_package(ament_cmake REQUIRED)

# 安装四套标准 playground 场景目录。
install(DIRECTORY 
  highway_lite 
  highway_v1.0 
  ring_small_v1.0 
  ring_tiny_v1.0 
  DESTINATION share/${PROJECT_NAME})


# 预留 data 资源安装规则；当前源码树没有该目录。
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  FILES_MATCHING PATTERN "*.json" PATTERN "*.xml"
)

# baseline 再次安装相同四场景目录，内容与上方规则重复。
install(DIRECTORY highway_lite highway_v1.0 ring_small_v1.0 ring_tiny_v1.0 DESTINATION share/${PROJECT_NAME})

ament_package()
