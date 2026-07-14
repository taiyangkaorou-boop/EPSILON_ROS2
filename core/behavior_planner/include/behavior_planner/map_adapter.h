#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_ADAPTER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_ADAPTER_H_

#include "behavior_planner/map_interface.h"
#include "common/basics/semantics.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace planning {

/**
 * @brief 将 SemanticMapManager 适配为 BehaviorPlannerMapItf 的薄转发层。
 *
 * 适配器通过 shared_ptr 共享地图生命周期，统一在未初始化时返回 kWrongStatus；少数
 * 查询还会复制/筛选 SemanticLaneSet 或补充有效性检查。
 */
class BehaviorPlannerMapAdapter : public BehaviorPlannerMapItf {
 public:
  /// 底层集成语义地图类型。
  using IntegratedMap = semantic_map_manager::SemanticMapManager;
  /// 返回 set_map 后缓存的有效标志。
  bool IsValid() override;
  /// 复制自车状态。
  ErrorType GetEgoState(State *state) override;
  /// 复制自车 ID。
  ErrorType GetEgoId(int *id) override;
  /// 复制完整自车对象。
  ErrorType GetEgoVehicle(common::Vehicle *vehicle) override;
  /// 使用自车位置查询最近 Lane ID。
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
  /// 查询右侧可换 Lane ID。
  ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) override;
  /// 查询左侧可换 Lane ID。
  ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) override;
  /// 复制纵向后继 Lane ID。
  ErrorType GetChildLaneIds(const int lane_id,
                            std::vector<int> *child_ids) override;
  /// 复制纵向前驱 Lane ID。
  ErrorType GetFatherLaneIds(const int lane_id,
                             std::vector<int> *father_ids) override;
  /// 按 ID 复制 Lane 并检查几何有效性。
  ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) override;
  /// 转发状态附近局部中心线采样。
  ErrorType GetLocalLaneSamplesByState(const State &state, const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) override;
  /// 转发按横向行为拼接参考 Lane 的查询。
  ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality, Lane *lane) override;
  /// 复制关键原始车辆集合。
  ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) override;
  /// 复制关键语义车辆集合。
  ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) override;
  /// 复制完整 LaneNet。
  ErrorType GetWholeLaneNet(common::LaneNet *lane_net) override;
  /// 转发两车状态几何碰撞检查。
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b,
                                     bool *res) override;
  /// 转发单车与地图/环境碰撞检查。
  ErrorType CheckIfCollision(const common::VehicleParam &vehicle_param,
                             const State &state, bool *res) override;
  /// 转发参考 Lane 前车搜索。
  ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio) override;
  /// 转发 Lane 上限速查询。
  ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                          decimal_t *speed_limit) override;
  /// 查询指定周车的预测横向行为。
  ErrorType GetPredictedBehavior(
      const int vehicle_id, common::LateralBehavior *lat_behavior) override;

  /// 设置底层地图共享指针并把适配器标记为有效；当前不拒绝空指针。
  void set_map(std::shared_ptr<IntegratedMap> map_ptr);

 private:
  /// 共享持有 SemanticMapManager。
  std::shared_ptr<IntegratedMap> map_;
  /// 由 set_map 单向置真，不随指针内容重新验证。
  bool is_valid_ = false;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_ADAPTER_H_
