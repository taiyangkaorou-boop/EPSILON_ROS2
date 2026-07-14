cmake_minimum_required(VERSION 3.5)
project(behavior_planner)

# MPDM 包使用 C++17，并无条件覆盖为 Release/-O3。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ROS2、地图、车辆模型、前向仿真和 RoutePlanner 依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(route_planner REQUIRED)

# 旧式全目录 include 路径。
include_directories(
  include
  ${common_INCLUDE_DIRS}
  ${vehicle_model_INCLUDE_DIRS}
  ${semantic_map_manager_INCLUDE_DIRS}
  ${forward_simulator_INCLUDE_DIRS}
  ${route_planner_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# MPDM 核心和地图适配器库。
add_library(behavior_planner_lib 
    src/behavior_planner/behavior_planner.cc
    src/behavior_planner/map_adapter.cc
)

target_link_libraries(behavior_planner_lib
    hkust_pl_common
    hkust_pl_rp
    hkust_pl_smm
    hkust_pl_vehicle_model
)

# ROS2 server/visualizer 封装库。
add_library(behavior_planner_ros 
    src/behavior_planner/behavior_server_ros.cc
)

target_link_libraries(behavior_planner_ros
    behavior_planner_lib
    hkust_pl_smm
)

# 导出两个库、公共依赖和 include 路径。
ament_export_libraries(behavior_planner_lib behavior_planner_ros)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    vehicle_model
    semantic_map_manager
    forward_simulator
    route_planner
)

ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
)

# 安装库和手写公共头。
install(TARGETS behavior_planner_lib behavior_planner_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
