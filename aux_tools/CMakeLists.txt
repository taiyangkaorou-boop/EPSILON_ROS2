cmake_minimum_required(VERSION 3.5)
project(aux_tools)

# 辅助工具包当前主要承载 Python HMI/可视化脚本；保留 C++17 配置用于后续工具扩展。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# 声明键盘控制、图像处理、坐标变换和自定义车辆消息所需依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# 为未来可能加入的 C++ 辅助节点准备依赖头文件路径。
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${vehicle_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 当前没有 C++ target；ament_package 负责注册 ROS2 包及其依赖元数据。
ament_package()


