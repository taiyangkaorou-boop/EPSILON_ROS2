cmake_minimum_required(VERSION 3.5)
project(semantic_map_manager)

# SMM 使用 C++17、详细 make 输出，并无条件覆盖为 Release/-O3。
set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# 预留包内自定义 Find 模块路径。
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ROS2/TF/消息、OpenCV/Eigen 和规划内部包依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(motion_predictor REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(vehicle_model REQUIRED)

# SMM 全 target 共用的源码/第三方/依赖 include 路径。
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${vehicle_msgs_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${motion_predictor_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# 地图渲染、配置、交通信号和语义地图核心库。
add_library(hkust_pl_smm
    src/semantic_map_manager.cc
    src/data_renderer.cc
    src/config_loader.cc
    src/traffic_signal_manager.cc
)
ament_target_dependencies(hkust_pl_smm
    rclcpp
    vehicle_msgs
    vehicle_model
    common
    motion_predictor
    Eigen3
    OpenCV
)

# ROS2 输入适配和可视化封装库。
add_library(hkust_smm_ros
    src/ros_adapter.cc
    src/visualizer.cc
)
ament_target_dependencies(hkust_smm_ros
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    sensor_msgs
    geometry_msgs
    Eigen3
    OpenCV
    common
    motion_predictor
    vehicle_msgs
    vehicle_model
)
target_link_libraries(hkust_smm_ros
    hkust_pl_smm
)

# 导出两个库、公共依赖和 include 路径。
ament_export_libraries(hkust_pl_smm hkust_smm_ros)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
    OpenCV
    common
    motion_predictor
    vehicle_msgs
    vehicle_model
)

ament_export_include_directories(
    include
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${vehicle_msgs_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${motion_predictor_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# 安装公共头。
install(DIRECTORY include/
  DESTINATION include
)

# 安装两个库并登记 export_hkust_pl_smm export set。
install(TARGETS hkust_pl_smm hkust_smm_ros
  EXPORT export_hkust_pl_smm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
