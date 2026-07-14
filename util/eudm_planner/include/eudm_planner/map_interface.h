#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_INTERFACE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_INTERFACE_H_

#include <iostream>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/**
 * @brief EUDM 规划核心依赖的只读语义地图抽象接口。
 *
 * 接口把 EudmPlanner 与 SemanticMapManager 的具体实现隔离，统一提供自车、车道拓扑、
 * 局部参考线、周车语义、碰撞和前后车查询。调用方通过 ErrorType 判断查询是否成功，
 * 查询结果由输出指针写回。
 */
class EudmPlannerMapItf {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;

  /// 返回适配器及其底层语义地图是否已准备好。
  virtual bool IsValid() = 0;
  /// 获取自车世界坐标状态。
  virtual ErrorType GetEgoState(State *state) = 0;
  /// 获取自车 ID。
  virtual ErrorType GetEgoId(int *id) = 0;
  /// 获取包含尺寸参数和状态的完整自车对象。
  virtual ErrorType GetEgoVehicle(common::Vehicle *vehicle) = 0;
  /// 在导航路径约束下查询自车位置对应的最近 Lane ID。
  virtual ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                           int *lane_id) = 0;
  /// 按平面位姿查询最近 Lane，并返回横向距离和弧长投影。
  virtual ErrorType GetNearestLaneIdUsingState(
      const Vec3f &state, const std::vector<int> &navi_path, int *id,
      decimal_t *distance, decimal_t *arc_len) = 0;
  /// 判断指定 Lane 能否沿拓扑到达路径，并返回所需换道次数。
  virtual ErrorType IsTopologicallyReachable(const int lane_id,
                                             const std::vector<int> &path,
                                             int *num_lane_changes,
                                             bool *res) = 0;
  /// 判断新旧 Lane 是否相同或存在有限深度的纵向后继关系。
  virtual bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) = 0;
  /// 获取规划器筛选后的关键原始车辆集合。
  virtual ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) = 0;
  /// 获取语义地图维护的全部周围车辆集合。
  virtual ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) = 0;
  /// 获取附带参考 Lane 和预测行为的关键语义车辆集合。
  virtual ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) = 0;
  /// 查询当前 Lane 可换入的右侧 Lane ID。
  virtual ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) = 0;
  /// 查询当前 Lane 可换入的左侧 Lane ID。
  virtual ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) = 0;
  /// 获取当前 Lane 的纵向后继 Lane ID。
  virtual ErrorType GetChildLaneIds(const int lane_id,
                                    std::vector<int> *child_ids) = 0;
  /// 获取当前 Lane 的纵向前驱 Lane ID。
  virtual ErrorType GetFatherLaneIds(const int lane_id,
                                     std::vector<int> *father_ids) = 0;
  /// 按 ID 获取连续 Lane 几何。
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) = 0;
  /// 截取状态附近、受导航路径约束的局部中心线样本。
  virtual ErrorType GetLocalLaneSamplesByState(
      const State &state, const int lane_id, const std::vector<int> &navi_path,
      const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
      vec_Vecf<2> *samples) = 0;
  /// 根据横向行为拼接给定前后长度的参考 Lane。
  virtual ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality,
      Lane *lane) = 0;
  /// 获取完整原始 LaneNet 副本。
  virtual ErrorType GetWholeLaneNet(common::LaneNet *lane_net) = 0;
  /// 检查两组车辆参数/状态表示的车身是否碰撞。
  virtual ErrorType CheckCollisionUsingState(
      const common::VehicleParam &param_a, const common::State &state_a,
      const common::VehicleParam &param_b, const common::State &state_b,
      bool *res) = 0;
  /// 在参考 Lane 和候选车辆集合中寻找前车及剩余搜索距离比例。
  virtual ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) = 0;
  /// 同时查询参考状态前后的最近车辆及其 Frenet 状态。
  virtual ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) = 0;
};

}  // namespace planning

#endif
