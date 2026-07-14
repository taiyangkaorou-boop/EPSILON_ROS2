cmake_minimum_required(VERSION 3.8)
project(ai_agent_planner)

# 使用 C++17；当前无条件覆盖 Release/-O3。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# 预留自定义 Find 模块路径；当前包内没有 cmake 目录。
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ROS2、规划/仿真内部库和消息依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
find_package(vehicle_msgs REQUIRED)


# 旧式全目录 include；rclcpp 由 ament_target_dependencies 注入。
include_directories(
    # ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}  
)

# 单一交通参与者可执行程序。
add_executable(onlane_ai_agent
    src/onlane_ai_agent.cc
)

# 声明 ament target 依赖。
ament_target_dependencies(onlane_ai_agent
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    vehicle_model
    forward_simulator
    semantic_map_manager
    behavior_planner
    vehicle_msgs
)

# 额外直接链接 Backward、OOQP、BLAS/MA27/Fortran 和旧式变量库。
target_link_libraries(onlane_ai_agent
    dw
    ${catkin_INCLUDE_DIRS}
    ${rclcpp_LIBRARIES}
    ${common_LIBRARIES}
    ${semantic_map_manager_LIBRARIES}
    ${vehicle_model_LIBRARIES}
    ${forward_simulator_LIBRARIES}
    ${behavior_planner_LIBRARIES}
    ${vehicle_msgs_LIBRARIES}
    ooqpgensparse
    ooqpsparse 
    ooqpgondzio 
    ooqpbase 
    blas 
    ma27 
    gfortran 
)

# 安装可执行文件到 ROS2 package lib 目录。
install(TARGETS
  onlane_ai_agent
  DESTINATION lib/${PROJECT_NAME})

# 同时安装 ROS2 Python launch 和遗留 ROS1 XML launch。
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
