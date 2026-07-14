cmake_minimum_required(VERSION 3.5)
project(vehicle_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 interface 生成、基础消息、rclcpp 和 common 编解码依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common REQUIRED)

# 公开 encoder/decoder 头需要 common include。
include_directories(
  include
  ${common_INCLUDE_DIRS}
)


# 生成场景、车辆、Lane、障碍物、状态、控制和占用栅格消息。
# MotionControl.msg 当前未列入生成集合。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/ControlSignal.msg"
  "msg/Lane.msg"
  "msg/LaneNet.msg"
  "msg/VehicleParam.msg"
  "msg/Vehicle.msg"
  "msg/VehicleSet.msg"
  "msg/OccupancyGridFloat.msg"
  "msg/OccupancyGridUInt8.msg"
  "msg/Circle.msg"
  "msg/CircleObstacle.msg"
  "msg/PolygonObstacle.msg"
  "msg/ObstacleSet.msg"
  "msg/FreeState.msg"
  "msg/ArenaInfo.msg"
  "msg/ArenaInfoDynamic.msg"
  "msg/ArenaInfoStatic.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
)

# 导出手写编解码头和 rosidl runtime。
ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
)
ament_export_dependencies(rosidl_default_runtime)


# 安装原始 msg schema 和手写 encoder/decoder 头。
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
