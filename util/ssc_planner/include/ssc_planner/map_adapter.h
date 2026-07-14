/**
 * @file map_adapter.h
 * @author HKUST Aerial Robotics Group
 * @brief 将 SemanticMapManager 快照适配为 SSC 地图接口。
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__

#include "common/basics/semantics.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "ssc_planner/map_interface.h"

namespace planning {

/**
 * @brief SscPlannerMapItf 在集成语义地图上的只读转发实现。
 *
 * ROS 服务端每次规划前复制一份 SemanticMapManager，并通过 set_map 绑定到本适配器。
 * 各 getter 再把该快照中的自车、行为结果、Lane 和障碍物数据复制给 SscPlanner。
 */
class SscPlannerAdapter : public SscPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;

  /// 返回内部显式有效标志。
  bool IsValid() override;

  /// 转发当前 SemanticMapManager 快照时间戳。
  decimal_t GetTimeStamp() override;

  /// 复制快照中的完整自车。
  ErrorType GetEgoVehicle(Vehicle* vehicle) override;

  /// 复制快照中的自车状态。
  ErrorType GetEgoState(State* state) override;

  /// 复制行为层参考车道。
  ErrorType GetEgoReferenceLane(Lane* lane) override;

  /// 复制供 SSC 使用的局部参考车道；当前实现与 ego reference lane 相同。
  ErrorType GetLocalReferenceLane(Lane* lane) override;

  /// 从 SemanticLaneSet 按 ID 复制 Lane。
  ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) override;

  /// 复制二维障碍栅格地图。
  ErrorType GetObstacleMap(GridMap2D* grid_map) override;

  /// 委托 SemanticMapManager 做静态/动态碰撞检查。
  ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                             const State& state, bool* res) override;

  /// 复制候选横向行为及自车前向 rollout。
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) override;

  /// 复制行为层最终选定的离散横向行为。
  ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) override;

  /// 复制候选行为、自车 rollout 和对应周车 rollout。
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs)
      override;

  /// 复制世界坐标障碍栅格中心集合。
  ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) override;

  /// 绑定新的集成地图共享快照，并把适配器标记为有效。
  ErrorType set_map(std::shared_ptr<IntegratedMap> map);

 private:
  /// 当前规划周期读取的 SemanticMapManager 共享快照。
  std::shared_ptr<IntegratedMap> map_;

  /// set_map 调用后置 true；当前实现不提供清空或失效接口。
  bool is_valid_ = false;
};

}  // namespace planning

#endif
