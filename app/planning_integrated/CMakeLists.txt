cmake_minimum_required(VERSION 3.5)
project(planning_integrated)

# 集成包不实现算法库，只编译 MPDM/EUDM 两个系统入口并连接既有规划组件。
# 统一使用 C++17，确保与各 ROS2 规划包的接口和标准库特性一致。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# ROS2 通信、地图、行为决策、轨迹规划以及数值求解依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
find_package(eudm_planner REQUIRED)
find_package(ssc_planner REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    # MPDM 前向仿真可使用 OpenMP；未找到时仍允许以单线程方式配置工程。
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 暴露依赖包的头文件路径，供两个集成入口直接组装服务器对象。
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    ${eudm_planner_INCLUDE_DIRS}
    ${ssc_planner_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)


# MPDM + SSC 对照入口：行为层使用 behavior_planner，运动层固定为 ssc_planner。
add_executable(test_ssc_with_mpdm src/test_ssc_with_mpdm.cc)
ament_target_dependencies(test_ssc_with_mpdm
  rclcpp
  visualization_msgs
  sensor_msgs
  message_filters
  common
  semantic_map_manager
  behavior_planner
  ssc_planner
  Protobuf
  glog
)

target_link_libraries(test_ssc_with_mpdm
  # backward-cpp、规划算法和 OOQP/BLAS/MA27 数值后端。
  dw
  ${behavior_planner_LIBRARIES}
  ${ssc_planner_LIBRARIES}
  ${Protobuf_LIBRARIES}
  glog::glog
  ${OpenMP_CXX_LIBRARIES} 
  ooqpgensparse
  ooqpsparse 
  ooqpgondzio 
  ooqpbase blas ma27 gfortran 
)

add_executable(test_ssc_with_eudm src/test_ssc_with_eudm.cc)
# EUDM + SSC 主研究入口：只替换行为决策层，保持运动层与 MPDM 对照一致。
ament_target_dependencies(test_ssc_with_eudm
  rclcpp
  visualization_msgs
  sensor_msgs
  message_filters
  common
  semantic_map_manager
  eudm_planner
  ssc_planner
  Protobuf
  glog
)

target_link_libraries(test_ssc_with_eudm
  # EUDM 入口当前未显式加入 OpenMP 库，数值求解后端与 MPDM 保持一致。
  dw
  ${ssc_planner_LIBRARIES}
  ${eudm_planner_LIBRARIES}
  ${Protobuf_LIBRARIES}
  glog::glog
  ooqpgensparse
  ooqpsparse 
  ooqpgondzio 
  ooqpbase blas ma27 gfortran 
)

# baseline 尝试导出运行/构建依赖；首个可执行 target 名也会被当作依赖包名处理。
ament_export_dependencies(test_ssc_with_eudm
    rclcpp
    visualization_msgs
    sensor_msgs
    message_filters
    common
    semantic_map_manager
    behavior_planner
    eudm_planner
    ssc_planner
    glog
    Protobuf
)

ament_export_include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    ${eudm_planner_INCLUDE_DIRS}
    ${ssc_planner_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)

# 历史安装形式保留作迁移参考；ROS2 ament 使用下方 lib/${PROJECT_NAME} 布局。
# install(TARGETS test_ssc_with_mpdm test_ssc_with_eudm
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(TARGETS 
  test_ssc_with_mpdm 
  test_ssc_with_eudm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 生成 ament 包索引和导出信息。
ament_package()
