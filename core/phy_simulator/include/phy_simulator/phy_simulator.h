/**
 * @file phy_simulator.h
 * @author HKUST Aerial Robotics Group
 * @brief 基于运动学自行车模型的多车物理仿真状态容器。
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_PHY_SIMULATOR_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_PHY_SIMULATOR_H_

#include <assert.h>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "phy_simulator/arena_loader.h"
#include "phy_simulator/basics.h"
#include "vehicle_model/vehicle_model.h"

namespace phy_simulator {

/**
 * @brief 加载 playground 场景并按车辆控制信号推进全部车辆状态。
 *
 * 每辆车对应一个 simulator::VehicleModel。闭环信号使用 steer_rate/acc 积分，标记为
 * openloop 的信号直接覆盖模型 State；类还支持在静态 ObstacleSet 中增删方形临时障碍物。
 */
class PhySimulation {
 public:
  /// 使用空路径 ArenaLoader 构造并立即尝试加载场景。
  PhySimulation();
  /// 使用三类场景文件加载数据并为所有车辆创建运动学模型。
  PhySimulation(const std::string &vehicle_set_path,
                 const std::string &map_path, const std::string &lane_net_path);
  /// 当前析构为空，不释放内部 new 的 ArenaLoader。
  ~PhySimulation() {}

  /// 按值返回完整 LaneNet。
  common::LaneNet lane_net() const { return lane_net_; }
  /// 按值返回静态及临时障碍物集合。
  common::ObstacleSet obstacle_set() const { return obstacle_set_; };
  /// 按值返回当前全部车辆状态和参数。
  common::VehicleSet vehicle_set() const { return vehicle_set_; }
  /// 按值返回加载车辆 ID；顺序来自 unordered_map 遍历。
  const std::vector<int> vehicle_ids() const { return vehicle_ids_; }

  /// 以 pt 为中心、size 为边长添加轴对齐方形临时障碍物。
  bool AddTemporaryObstacleToMap(const common::Point &pt, const double &size);
  /// 删除中心到 pt 距离小于 s 的全部临时障碍物。
  bool RemoveTemporaryObstacle(const common::Point &pt, const double &s);

  /// 使用同一 dt 更新全部车辆；当前接口固定报告成功。
  bool UpdateSimulatorUsingSignalSet(
      const common::VehicleControlSignalSet &signal_set, const decimal_t &dt);

 private:
  /// 调用 ArenaLoader 依次加载车辆、障碍物和 LaneNet。
  bool GetDataFromArenaLoader();

  /// 为 VehicleSet 中每辆车创建并初始化一个 VehicleModel。
  bool SetupVehicleModelForVehicleSet();

  /// 按车辆 ID 读取控制信号并积分或直接覆盖状态。
  bool UpdateVehicleStates(const common::VehicleControlSignalSet &signal_set,
                           const decimal_t &dt);

  /// 由构造函数 new 的场景加载器裸指针。
  ArenaLoader *p_arena_loader_;

  // 当前车辆、对应运动学模型及对外 ID 列表。
  common::VehicleSet vehicle_set_;
  std::unordered_map<int, simulator::VehicleModel> vehicle_model_set_;
  std::vector<int> vehicle_ids_;

  // 静态道路拓扑和障碍物环境。
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;

  // 临时障碍物使用 `10000+递增计数` 生成 ID，并另存一份用于空间删除。
  int temp_obstacle_cnt_ = 0;
  int temp_obs_idx_offset_ = 10000;
  std::unordered_map<int, common::PolygonObstacle> temp_obstacle_set_;
};

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_PHY_SIMULATOR_H_
