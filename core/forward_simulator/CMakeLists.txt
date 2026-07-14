cmake_minimum_required(VERSION 3.5)
project(forward_simulator)

# 头文件前向仿真包使用 C++17，并无条件覆盖为 Release/-O3。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# 预留包内自定义 Find 模块路径。
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# 当前声明 ROS2、消息、common 和 vehicle_model 依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)

# 公开前向仿真头所需 include 路径。
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
)

# 本包没有自建 library，当前转而导出 vehicle_model 的库变量。
ament_export_libraries(${vehicle_model_LIBRARIES})

ament_export_dependencies(
  rclcpp 
  visualization_msgs 
  sensor_msgs 
  common 
  vehicle_model
)


# 导出手写头和 vehicle_model include，并安装 include 目录。
ament_export_include_directories(
    include
    ${vehicle_model_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
