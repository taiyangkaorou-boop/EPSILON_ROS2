cmake_minimum_required(VERSION 3.5)
project(ssc_planner)

# 统一使用 C++17；调用者未指定标准时才设置默认值。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# 将包内 cmake 目录加入模块搜索路径；当前源码树中该目录不存在。
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ROS2 通信、语义地图、消息、OpenMP、protobuf 和日志依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(Protobuf REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(Glog REQUIRED)

# 从 proto2 schema 生成 ssc_config.pb.cc/.h，输出位于当前 binary 目录。
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/ssc_config.proto
)

# 安装运行时文本配置，launch 文件通过包 share 目录解析该路径。
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# 为本包两个库及其下游暴露源码头文件、生成头文件和依赖头路径。
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

# 算法库：地图适配、SSC 地图、规划核心和生成的 protobuf 实现。
add_library(hkust_pl_ssc
    src/ssc_planner/ssc_planner.cc
    src/ssc_planner/ssc_map.cc
    src/ssc_planner/map_adapter.cc
    ${PROTO_SRCS}
)
target_link_libraries(hkust_pl_ssc
    hkust_pl_common
    hkust_pl_smm
    OpenMP::OpenMP_CXX
    ${Protobuf_LIBRARIES}
    glog::glog
)

# ROS2 包装库：后台规划服务端和 Marker 可视化器。
add_library(ssc_server_ros
    src/ssc_planner/ssc_server_ros.cc
    src/ssc_planner/ssc_visualizer.cc
)
target_link_libraries(ssc_server_ros
    hkust_pl_ssc
    hkust_smm_ros
    OpenMP::OpenMP_CXX
)

# 向下游 ament 包导出依赖、头路径和两个库名。
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    semantic_map_manager
    Protobuf
    OpenMP
    vehicle_msgs
    glog
)

ament_export_include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
)

ament_export_libraries(hkust_pl_ssc ssc_server_ros)

# 安装共享/静态库和运行时目标。
install(TARGETS hkust_pl_ssc ssc_server_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装源码 include 树；生成的 ssc_config.pb.h 当前未包含在此规则中。
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
