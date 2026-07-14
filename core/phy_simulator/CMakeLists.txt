cmake_minimum_required(VERSION 3.5)
project(phy_simulator)

# 调用者未指定标准时默认使用 C++17，并强制标准要求。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# ROS2 通信/消息、Eigen、common、车辆模型和控制消息依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(vehicle_msgs REQUIRED)

# 全局暴露本包和依赖头路径；semantics_msgs 变量当前没有对应 find_package。
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${semantics_msgs_INCLUDE_DIRS}
)

# 算法库只包含 ArenaLoader 和 PhySimulation，不包含 ROS2 adapter/visualizer。
add_library(phy_simulator_lib
    src/phy_simulator/arena_loader.cc
    src/phy_simulator/phy_simulator.cc
)

target_link_libraries(phy_simulator_lib
    # backward-cpp 的 Linux 栈回溯库及两个内部规划库。
    dw
    ${common_LIBRARIES}
    ${vehicle_model_LIBRARIES}
)

# ROS2 可执行程序同时编译 main、Visualizer 和 RosAdapter。
add_executable(phy_simulator_planning_node
    src/phy_simulator_planning_node.cc
    src/phy_simulator/visualizer.cc
    src/phy_simulator/ros_adapter.cc
)

target_link_libraries(phy_simulator_planning_node
    dw
    phy_simulator_lib
    ${common_LIBRARIES}
    ${vehicle_model_LIBRARIES}
    ${vehicle_msgs_LIBRARIES}
)

# 安装算法库和 ROS2 节点；可执行文件按 ament 约定放入 lib/<package>。
install(TARGETS phy_simulator_lib phy_simulator_planning_node
  EXPORT export_phy_simulator
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# 安装全部公开头文件。
install(DIRECTORY include/
  DESTINATION include
)

# 安装 ROS1/ROS2 launch 目录，供包共享路径查询。
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 向下游导出算法库名、依赖和头文件路径。
ament_export_libraries(phy_simulator_lib)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    Eigen3
    common
    vehicle_model
    vehicle_msgs
)

ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

ament_package()
