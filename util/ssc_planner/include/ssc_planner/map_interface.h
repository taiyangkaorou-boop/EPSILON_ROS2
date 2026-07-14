/**
 * @file map_interface.h
 * @author HKUST Aerial Robotics Group
 * @brief SSC 规划器使用的环境快照抽象接口。
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__

#include <array>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/**
 * @brief 隔离 SSC 核心算法与具体语义地图实现的数据访问契约。
 *
 * 规划器通过该接口获取同一环境快照中的自车、参考车道、静态障碍物、行为层前向轨迹及
 * 离散横向行为。接口只描述读取能力，不负责地图更新、对象所有权或线程同步。
 */
class SscPlannerMapItf {
 public:
  /// 障碍栅格单元的数据类型，与 SemanticMapManager 的二维 GridMap 保持一致。
  using ObstacleMapType = uint8_t;
  using State = common::State;
  using Lane = common::Lane;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using Behavior = common::SemanticBehavior;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  /// 查询当前适配器是否已绑定可供规划使用的地图快照。
  virtual bool IsValid() = 0;

  /// 返回当前环境快照的时间戳。
  virtual decimal_t GetTimeStamp() = 0;

  /// 复制当前自车的完整车辆参数和状态。
  virtual ErrorType GetEgoVehicle(Vehicle* vehicle) = 0;

  /// 只复制当前自车状态。
  virtual ErrorType GetEgoState(State* state) = 0;

  /// 返回行为层为自车生成的参考车道。
  virtual ErrorType GetEgoReferenceLane(Lane* lane) = 0;

  /// 返回供 SSC 建立 Frenet 坐标的局部参考车道。
  virtual ErrorType GetLocalReferenceLane(Lane* lane) = 0;

  /// 按语义 Lane ID 查询拟合后的中心线对象。
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) = 0;

  /// 复制当前二维静态障碍栅格地图。
  virtual ErrorType GetObstacleMap(GridMap2D* grid_map) = 0;

  /// 检查指定车辆几何和状态是否与当前地图环境发生碰撞。
  virtual ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                                     const State& state, bool* res) = 0;

  /// 返回行为层全部候选横向行为及对应自车前向 rollout。
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) = 0;

  /// 同时返回每个候选行为下的周车预测 rollout。
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<Vehicle>>>* sur_trajs) = 0;

  /// 返回行为层最终选定的离散横向行为。
  virtual ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) = 0;

  /// 复制以世界坐标保存的障碍栅格中心集合。
  virtual ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) = 0;
};

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__
