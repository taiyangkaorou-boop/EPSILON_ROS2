/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief SemanticMapManager 的轻量基础配置类型。
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_

#include <assert.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <vector>
#include <memory>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

namespace semantic_map_manager {

/**
 * @brief 单个 ego 对应的语义地图构建与预测开关配置。
 *
 * 配置由 ConfigLoader 按 ego ID 从 JSON 中填充，随后由 SemanticMapManager 用于障碍物
 * 栅格、周车筛选、预测模式、跟踪噪声、日志和快速 Lane 查表控制。
 */
struct AgentConfigInfo {
  /// 自车局部障碍物 GridMap 的宽、高和分辨率。
  common::GridMapMetaInfo obstacle_map_meta_info;

  /// 纳入 surrounding_vehicles 的平面搜索半径。
  decimal_t surrounding_search_radius;

  /// 是否使用开环车辆行为预测。
  bool enable_openloop_prediction{false};

  /// 是否向跟踪车辆状态注入模拟噪声。
  bool enable_tracking_noise{false};

  /// 是否把语义地图运行数据写入日志文件。
  bool enable_log{false};

  /// 是否启用快速 Lane 查询表；ConfigLoader 当前未从 JSON 覆盖此默认值。
  bool enable_fast_lane_lut{true};

  /// enable_log 对应的输出文件路径。
  std::string log_file;

  /// 依次打印栅格元信息、搜索半径、功能开关和日志路径。
  void PrintInfo() {
    obstacle_map_meta_info.print();
    printf("surrounding_search_radius: %f\n", surrounding_search_radius);
    printf("enable_openloop_prediction: %d\n", enable_openloop_prediction);
    printf("enable_tracking_noise: %d\n", enable_tracking_noise);
    printf("enable_log: %d\n", enable_log);
    printf("enable_fast_lane_lut: %d\n", enable_fast_lane_lut);
    printf("log_file: %s\n", log_file.c_str());
  }
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_
