/**
 * @file map_adapter.cc
 * @author HKUST Aerial Robotics Group
 * @brief SSC 语义地图适配器实现。
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include "ssc_planner/map_adapter.h"

namespace planning {

ErrorType SscPlannerAdapter::set_map(std::shared_ptr<IntegratedMap> map) {
  // 保存服务端提供的环境共享快照；后续所有 getter 都从同一对象读取。
  map_ = map;
  // 原始 baseline 只使用独立标志表示有效，不检查传入 shared_ptr 是否为空。
  is_valid_ = true;
  return kSuccess;
}

// 返回适配器显式状态，不进一步核对 map_ 指针或地图内部状态。
bool SscPlannerAdapter::IsValid() { return is_valid_; }

// 时间戳接口直接转发 map_；调用者应先保证已经绑定地图快照。
decimal_t SscPlannerAdapter::GetTimeStamp() { return map_->time_stamp(); }

ErrorType SscPlannerAdapter::GetEgoVehicle(Vehicle* vehicle) {
  if (!is_valid_) return kWrongStatus;
  // SemanticMapManager getter 按值返回，再复制到调用者输出对象。
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetEgoState(State* state) {
  if (!is_valid_) return kWrongStatus;
  // 通过完整自车值拷贝取得其 State。
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetLocalReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  // 当前局部参考车道直接使用行为层 SemanticBehavior.ref_lane。
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  // Lane 可能包含样条和采样缓存，此处执行完整值拷贝。
  *lane = ref_lane;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs) {
  if (!is_valid_) return kWrongStatus;
  // 至少存在一个候选行为才认为行为层 rollout 可供 SSC 使用。
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  // 两个容器依赖相同下标对应同一候选行为，当前适配层不检查长度一致性。
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs) {
  if (!is_valid_) return kWrongStatus;
  // 三输出版本沿用相同的“候选行为非空”可用性判断。
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  // 行为、自车轨迹和周车轨迹均从 SemanticBehavior 深拷贝给规划器。
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  *sur_trajs = map_->ego_behavior().surround_trajs;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetEgoDiscretBehavior(
    LateralBehavior* lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  // kUndefined 表示行为层尚未给出可执行横向决策。
  if (map_->ego_behavior().lat_behavior == common::LateralBehavior::kUndefined)
    return kWrongStatus;
  *lat_behavior = map_->ego_behavior().lat_behavior;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetLaneByLaneId(const int lane_id, Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  // semantic_lane_set() 按值返回，因此该查询会先复制完整语义 Lane 集合。
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // SemanticLane 还包含拓扑元数据，本接口只向 SSC 暴露中心线 Lane。
    *lane = it->second.lane;
  }
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetEgoReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  // 与 GetLocalReferenceLane 相同，直接取行为层生成的 ref_lane。
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetObstacleMap(GridMap2D* grid_map) {
  if (!is_valid_) return kWrongStatus;
  // obstacle_map() 和本次赋值都会产生值拷贝，规划器随后持有独立栅格。
  *grid_map = map_->obstacle_map();
  return kSuccess;
}

ErrorType SscPlannerAdapter::CheckIfCollision(
    const common::VehicleParam& vehicle_param, const State& state, bool* res) {
  if (!is_valid_) return kWrongStatus;
  // 将车辆几何和候选状态交给集成地图；原实现不转发其 ErrorType。
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetObstacleGrids(
    std::set<std::array<decimal_t, 2>>* obs_grids) {
  if (!is_valid_) return kWrongStatus;
  // 复制跨帧累积的世界坐标障碍栅格中心集合。
  *obs_grids = map_->obstacle_grids();
  return kSuccess;
}

}  // namespace planning
