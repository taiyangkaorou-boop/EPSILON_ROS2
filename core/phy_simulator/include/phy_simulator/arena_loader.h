/**
 * @file arena_loader.h
 * @author HKUST Aerial Robotics Group
 * @brief 从场景 JSON/GeoJSON 文件加载车辆、障碍物和 LaneNet。
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

#include "phy_simulator/basics.h"

namespace phy_simulator {

/**
 * @brief 将 playground 场景文件转换为 common 仿真语义对象。
 *
 * vehicle_set_path 指向自车/周车初始状态与车辆参数 JSON，map_path 指向障碍物
 * MultiPolygon GeoJSON，lane_net_path 指向 Lane 中心线及拓扑 MultiLineString GeoJSON。
 * 类只负责文件路径和解析，不拥有输出 VehicleSet/ObstacleSet/LaneNet。
 */
class ArenaLoader {
 public:
  /// 默认构造保留三个空路径。
  ArenaLoader();

  /// 使用车辆、障碍地图和 LaneNet 三个文件路径构造加载器。
  ArenaLoader(const std::string &vehicle_set_path, const std::string &map_path,
              const std::string &lane_net_path);

  /// 按值返回当前车辆配置文件路径。
  inline std::string vehicle_set_path() const { return vehicle_set_path_; }
  /// 按值返回当前障碍地图文件路径。
  inline std::string map_path() const { return map_path_; }
  /// 按值返回当前 LaneNet 文件路径。
  inline std::string lane_net_path() const { return lane_net_path_; }

  /// 覆盖车辆配置文件路径。
  inline void set_vehicle_set_path(const std::string &path) {
    vehicle_set_path_ = path;
  }
  /// 覆盖障碍地图文件路径。
  inline void set_map_path(const std::string &path) { map_path_ = path; }
  /// 覆盖 LaneNet 文件路径。
  inline void set_lane_net_path(const std::string &path) {
    lane_net_path_ = path;
  }

  /// 解析 vehicles.info，构造车辆 ID、类型、初始 State 和 VehicleParam。
  bool ParseVehicleSet(common::VehicleSet *p_vehicle_set);

  /// 解析有效障碍 Feature 的首个 MultiPolygon 外环并写入 ObstacleSet。
  ErrorType ParseMapInfo(common::ObstacleSet *p_obstacle_set);

  /// 解析 Lane Feature 的属性、拓扑字符串和首条中心线并写入 LaneNet。
  ErrorType ParseLaneNetInfo(common::LaneNet *p_lane_net);

 private:
  // 三类场景输入路径。
  std::string vehicle_set_path_;
  std::string map_path_;
  std::string lane_net_path_;
  /// 预留的行人配置路径；当前无 setter/getter 和解析逻辑。
  std::string pedestrian_set_path_;
};

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_
