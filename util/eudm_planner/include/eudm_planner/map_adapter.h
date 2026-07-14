#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_

#include <iostream>
#include <set>

#include "common/basics/semantics.h"
#include "eudm_planner/map_interface.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace planning {

/**
 * @brief 将 SemanticMapManager 适配为 EudmPlannerMapItf 的薄转发层。
 *
 * 适配器通过 shared_ptr 共享底层地图生命周期，大部分查询先检查 is_valid_，再复制底层
 * 数据或转发几何/拓扑查询。EudmPlanner 只保存本对象的非拥有接口指针。
 */
class EudmPlannerMapAdapter : public EudmPlannerMapItf {
 public:
  /// 底层集成语义地图类型。
  using IntegratedMap = semantic_map_manager::SemanticMapManager;
  /// 返回 set_map 是否已把适配器标记为有效。
  bool IsValid() override;
  /// 复制自车状态。
  ErrorType GetEgoState(State *state) override;
  /// 复制自车 ID。
  ErrorType GetEgoId(int *id) override;
  /// 复制完整自车对象。
  ErrorType GetEgoVehicle(common::Vehicle *vehicle) override;
  /// 使用自车当前位姿查询最近 Lane ID。
  ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                   int *lane_id) override;
  /// 转发任意平面位姿的最近 Lane 查询。
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) override;
  /// 转发 Lane 到导航路径的拓扑可达性查询。
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) override;
  /// 用最多 20 个扩展节点的后继 BFS 判断 Lane 连续性。
  bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) override;
  /// 从 SemanticLaneSet 查询右侧可换 Lane。
  ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) override;
  /// 从 SemanticLaneSet 查询左侧可换 Lane。
  ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) override;
  /// 复制纵向后继 Lane ID。
  ErrorType GetChildLaneIds(const int lane_id,
                            std::vector<int> *child_ids) override;
  /// 复制纵向前驱 Lane ID。
  ErrorType GetFatherLaneIds(const int lane_id,
                             std::vector<int> *father_ids) override;
  /// 从 SemanticLaneSet 复制指定 Lane，并检查几何有效性。
  ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) override;
  /// 转发局部中心线采样查询。
  ErrorType GetLocalLaneSamplesByState(const State &state, const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) override;
  /// 转发按横向行为构造参考 Lane 的查询，并检查结果有效性。
  ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality, Lane *lane);
  /// 复制关键原始车辆集合。
  ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) override;
  /// 复制全部周围车辆集合。
  ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) override;
  /// 复制关键语义车辆集合。
  ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) override;
  /// 复制完整原始 LaneNet。
  ErrorType GetWholeLaneNet(common::LaneNet *lane_net) override;
  /// 转发两车状态碰撞检查。
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b,
                                     bool *res) override;
  /// 转发参考 Lane 上的前车搜索。
  ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio) override;
  /// 转发参考 Lane 上前后车及 Frenet 状态查询。
  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) override;

  /// 设置底层地图共享指针，并把 is_valid_ 单向置为 true。
  void set_map(std::shared_ptr<IntegratedMap> map_ptr);

  /// 按值返回底层地图 shared_ptr，供 EudmManager 直接访问扩展接口。
  std::shared_ptr<IntegratedMap> map() { return map_; }

 private:
  /// 共享持有 SemanticMapManager。
  std::shared_ptr<IntegratedMap> map_;
  /// 由 set_map 置真，当前不随 map_ 是否为空重新验证。
  bool is_valid_ = false;
};

}  // namespace planning

#endif
