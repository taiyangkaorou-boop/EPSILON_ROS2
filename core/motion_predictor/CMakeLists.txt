cmake_minimum_required(VERSION 3.5)
project(motion_predictor)

# 头文件运动预测包使用 C++17/-fPIC，并无条件覆盖为 Release/-O3。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall -fPIC)

# 预留包内自定义 Find 模块路径。
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# 当前声明 ROS2、消息、common、vehicle_model 和 forward_simulator 依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)


# 公开预测头所需 include 路径。
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

# 本包没有自建 library，当前转而导出 forward_simulator 的库变量。
ament_export_libraries(${forward_simulator_LIBRARIES})

# 导出传递依赖。
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    vehicle_model
    forward_simulator
)

# 导出源码/依赖 include 路径。
ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
)

# 安装手写 header-only 实现。
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
