cmake_minimum_required(VERSION 3.5)
project(eudm_planner)

# 全包使用 C++17；当前还会无条件覆盖为 Release/-O3。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ROS2、规划基础库、protobuf 和 glog 依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(route_planner REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(Glog REQUIRED)

# 从 proto2 配置生成 eudm_config.pb.cc/.h；输出位于构建目录。
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS 
    proto/eudm_config.proto 
)

# 安装运行时 textproto 配置。
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# 旧式全目录 include 路径，同时暴露 protobuf 生成头所在构建目录。
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

# 与 ROS 无关的 EUDM 动作树、manager、地图适配和规划核心库。
add_library(hkust_pl_eudm
    src/eudm_planner/eudm_planner.cc
    src/eudm_planner/map_adapter.cc
    src/eudm_planner/eudm_manager.cc
    src/eudm_planner/dcp_tree.cc
    ${PROTO_SRCS}
)
# 直接链接 SemanticMapManager 内部 target、protobuf 和 glog。
target_link_libraries(hkust_pl_eudm
    hkust_pl_smm
    ${Protobuf_LIBRARIES}
    glog::glog
)

# ROS2 server/visualizer 封装库。
add_library(hkust_eudm_ros
    src/eudm_planner/eudm_server_ros.cc
)
target_link_libraries(hkust_eudm_ros
    hkust_pl_eudm
    hkust_smm_ros
    ${Protobuf_LIBRARIES}
    glog::glog
)

# 向下游导出两个库名。
ament_export_libraries(hkust_pl_eudm hkust_eudm_ros)

# 导出声明的公共依赖；TF 系列当前未在此显式列出。
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    Protobuf
    common
    vehicle_model
    semantic_map_manager
    forward_simulator
    route_planner
    vehicle_msgs
    glog
)

# 导出源码 include 目录及生成头的构建目录。
ament_export_include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
)

# 安装核心/ROS 库。
install(TARGETS hkust_pl_eudm hkust_eudm_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装手写公共头；当前未单独安装 eudm_config.pb.h。
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
