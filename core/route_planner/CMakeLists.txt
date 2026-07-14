cmake_minimum_required(VERSION 3.5)
project(route_planner)

# 实际使用 C++17；注释中的历史 C++14 已移除，构建仍强制 Release/-O3。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# 当前声明 ROS2、消息、common 和 SMM 依赖；源码实际只直接使用 common。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)

# 旧式全目录 include 路径。
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
)

# RoutePlanner 实现库。
add_library(hkust_pl_rp 
    src/route_planner/route_planner.cc
)

# 直接链接 common/SMM 工作区内部 target。
target_link_libraries(hkust_pl_rp
    hkust_pl_common
    hkust_pl_smm
)

# 导出库、依赖和公共 include 路径。
ament_export_libraries(hkust_pl_rp)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    semantic_map_manager
)

ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
)

# 安装库和手写公共头。
install(TARGETS hkust_pl_rp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
