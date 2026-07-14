cmake_minimum_required(VERSION 3.5)
project(vehicle_model)

# 实际使用 C++17，并无条件覆盖为 Release/-O3。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# ROS2/消息、Eigen/Boost 和 common 依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(Boost REQUIRED)

set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)

# 车辆模型 target 共用的源码、依赖和 thirdparty include。
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${THIRD_PARTY_INCLUDE_DIRS}
)

# 车辆运动学模型、IDM/CTX-IDM、pure-pursuit/PID/速度控制器库。
add_library(hkust_pl_vehicle_model 
    src/vehicle_model/vehicle_model.cc
    src/vehicle_model/ideal_steer_model.cc
    src/vehicle_model/idm_model.cc
    src/vehicle_model/ctx_idm_model.cc
    src/vehicle_model/controllers/pure_pursuit_controller.cc
    src/vehicle_model/controllers/pid_controller.cc
    src/vehicle_model/controllers/idm_velocity_controller.cc
    src/vehicle_model/controllers/ctx_idm_velocity_controller.cc
)


target_link_libraries(hkust_pl_vehicle_model
    hkust_pl_common
    ${Boost_LIBRARIES}
)

ament_export_libraries(hkust_pl_vehicle_model)
# 导出公共依赖和 include 路径。
ament_export_dependencies(
    rclcpp 
    visualization_msgs 
    sensor_msgs 
    geometry_msgs 
    Eigen3 
    common
    Boost
)

ament_export_include_directories(
    include 
    ${common_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${THIRD_PARTY_INCLUDE_DIRS}
)

# 给实际 target 注入 ament 依赖。
ament_target_dependencies(hkust_pl_vehicle_model
    rclcpp
    visualization_msgs 
    sensor_msgs 
    geometry_msgs 
    Eigen3 
    common
    Boost
)


install(DIRECTORY include/
  DESTINATION include
)

# 安装公共头和车辆模型库。
install(TARGETS hkust_pl_vehicle_model
  EXPORT export_hkust_pl_vehicle_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
