cmake_minimum_required(VERSION 3.5)
project(common)

# 公共算法库使用 C++17，并无条件覆盖为 Release/-O3。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# 使用包内 cmake 查找 OOQP 等非标准依赖。
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ROS2、TF、数值、日志、图像、protobuf 和 OOQP 依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OOQP REQUIRED)

if(NOT OOQP_FOUND)
    message(FATAL_ERROR "OOQP not found.")
endif()

# 源码树 thirdparty 和 OOQP 头路径。
set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty ${PROTOBUF_INCLUDE_DIR})
set(OOQP_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/OOQP/include)

# 旧式全目录 include，向本包所有 target 注入依赖头。
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/thirdparty
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${OOQP_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# OOQP 库目录和公共第三方链接集合。
link_directories(${OOQP_LIBRARY_DIRS})

set(THIRD_PARTY_LIBRARIES
    dw
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OOQP_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
)

# QP solver/OOQP 适配库。
add_library(hkust_pl_ooqp_itf 
    src/common/solver/ooqp_interface.cc
    src/common/solver/qp_solver.cc
)
target_link_libraries(hkust_pl_ooqp_itf 
    ${THIRD_PARTY_LIBRARIES}
    ooqpgensparse
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran 
)

# common 基础几何、Lane、状态变换、IDM/MOBIL/RSS、样条和 primitive 源文件。
set(COMMON_SRC
    src/common/basics/colormap.cc
    src/common/basics/semantics.cc
    src/common/basics/shapes.cc
    src/common/basics/tool_func.cc
    src/common/math/calculations.cc
    src/common/rss/rss_checker.cc
    src/common/idm/intelligent_driver_model.cc
    src/common/mobil/mobil_model.cc
    src/common/mobil/mobil_behavior_prediction.cc
    src/common/spline/lookup_table.cc
    src/common/spline/spline_generator.cc
    src/common/lane/lane.cc
    src/common/lane/lane_generator.cc
    src/common/primitive/frenet_primitive.cc
    src/common/state/state_transformer.cc
    src/common/state/free_state.cc
    src/common/circle_arc/circle_arc.cc
    src/common/circle_arc/circle_arc_branch.cc
)

add_library(hkust_pl_common 
    ${COMMON_SRC}
)
target_link_libraries(hkust_pl_common
    ${THIRD_PARTY_LIBRARIES}
    opencv_core
    opencv_highgui
    hkust_pl_ooqp_itf
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_geometry_msgs_LIBRARIES}
)

# 导出 common/OOQP 库、公共依赖和 include 路径。
ament_export_libraries(hkust_pl_common hkust_pl_ooqp_itf)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    tf2_ros
    Eigen3
    Glog
    OpenCV
    Protobuf
    tf2
    tf2_ros
    tf2_geometry_msgs
)

ament_export_include_directories(
    include
    ${THIRD_PARTY_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OOQP_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# 安装两个库并写入 hkust_pl_export export set。
install(TARGETS hkust_pl_common hkust_pl_ooqp_itf
  EXPORT hkust_pl_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装 CMake export 文件和可视化材料资源。
install(EXPORT hkust_pl_export
  FILE hkust_pl_export.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY materials/
  DESTINATION share/${PROJECT_NAME}/materials
)

ament_package()
