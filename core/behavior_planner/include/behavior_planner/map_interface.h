#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/**
 * @brief BehaviorPlanner 依赖的只读语义地图抽象接口。
 *
 * 接口隔离规划算法与 SemanticMapManager 具体实现，覆盖自车、车道拓扑、参考 Lane、
 * 周车、碰撞、限速和行为预测查询。输出对象多按值复制，调用方通过 ErrorType 判断失败。
 */
class BehaviorPlannerMapItf {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;

  /// 返回适配器及底层地图是否可用。
  virtual bool IsValid() = 0;
  /// 获取自车世界状态。
  virtual ErrorType GetEgoState(State *state) = 0;
  /// 获取自车 ID。
  virtual ErrorType GetEgoId(int *id) = 0;
  /// 获取完整自车对象。
  virtual ErrorType GetEgoVehicle(common::Vehicle *vehicle) = 0;
  /// 在导航路径约束下查询自车所在最近 Lane ID。
  virtual ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                           int *lane_id) = 0;
  /// 按三自由度状态查询最近 Lane、距离和弧长投影。
  virtual ErrorType GetNearestLaneIdUsingState(
      const Vec3f &state, const std::vector<int> &navi_path, int *id,
      decimal_t *distance, decimal_t *arc_len) = 0;
  /// 判断 Lane 是否可沿拓扑到达路径，并返回所需换道次数。
  virtual ErrorType IsTopologicallyReachable(const int lane_id,
                                             const std::vector<int> &path,
                                             int *num_lane_changes,
                                             bool *res) = 0;
  /// 获取规划关注的关键原始车辆集合。
  virtual ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) = 0;
  /// 获取带 Lane/预测语义的关键车辆集合。
  virtual ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) = 0;
  /// 查询可换入的右侧 Lane ID。
  virtual ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) = 0;
  /// 查询可换入的左侧 Lane ID。
  virtual ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) = 0;
  /// 获取纵向后继 Lane ID。
  virtual ErrorType GetChildLaneIds(const int lane_id,
                                    std::vector<int> *child_ids) = 0;
  /// 获取纵向前驱 Lane ID。
  virtual ErrorType GetFatherLaneIds(const int lane_id,
                                     std::vector<int> *father_ids) = 0;
  /// 按 ID 获取连续 Lane 几何。
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) = 0;
  /// 截取状态附近、受导航路径约束的局部中心线样本。
  virtual ErrorType GetLocalLaneSamplesByState(
      const State &state, const int lane_id, const std::vector<int> &navi_path,
      const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
      vec_Vecf<2> *samples) = 0;
  /// 根据横向行为拼接指定前后长度的参考 Lane。
  virtual ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality,
      Lane *lane) = 0;
  /// 获取完整原始 LaneNet 副本。
  virtual ErrorType GetWholeLaneNet(common::LaneNet *lane_net) = 0;
  /// 检查两组车辆参数/状态之间的车身碰撞。
  virtual ErrorType CheckCollisionUsingState(
      const common::VehicleParam &param_a, const common::State &state_a,
      const common::VehicleParam &param_b, const common::State &state_b,
      bool *res) = 0;
  /// 检查给定车辆状态与地图/周车环境是否碰撞。
  virtual ErrorType CheckIfCollision(const common::VehicleParam &vehicle_param,
                                     const State &state, bool *res) = 0;
  /// 在参考 Lane 和车辆集合中寻找前车及剩余距离比例。
  virtual ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) = 0;
  /// 查询状态在 Lane 上适用的速度限制。
  virtual ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                                  decimal_t *speed_limit) = 0;
  /// 查询指定周车当前预测的横向行为。
  virtual ErrorType GetPredictedBehavior(
      const int vehicle_id, common::LateralBehavior *lat_behavior) = 0;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_
