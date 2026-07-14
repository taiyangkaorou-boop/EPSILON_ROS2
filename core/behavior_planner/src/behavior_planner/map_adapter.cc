#include "behavior_planner/map_adapter.h"

namespace planning {

// 返回 set_map 维护的浅层有效标记。
bool BehaviorPlannerMapAdapter::IsValid() { return is_valid_; }

// 复制底层自车当前状态。
ErrorType BehaviorPlannerMapAdapter::GetEgoState(State *state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

// 返回底层地图保存的自车 ID。
ErrorType BehaviorPlannerMapAdapter::GetEgoId(int *id) {
  if (!is_valid_) return kWrongStatus;
  *id = map_->ego_id();
  return kSuccess;
}

// 复制完整自车对象。
ErrorType BehaviorPlannerMapAdapter::GetEgoVehicle(common::Vehicle *vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

// 用自车三自由度状态委托最近 Lane 查询，只返回 Lane ID。
ErrorType BehaviorPlannerMapAdapter::GetEgoLaneIdByPosition(
    const std::vector<int> &navi_path, int *lane_id) {
  if (!is_valid_) {
    printf("[GetEgoLaneIdByPosition]Interface not valid.\n");
    return kWrongStatus;
  }

  int ego_lane_id = kInvalidLaneId;
  decimal_t distance_to_lane;
  decimal_t arc_len;

  Vec3f state_3dof(map_->ego_vehicle().state().vec_position(0),
                   map_->ego_vehicle().state().vec_position(1),
                   map_->ego_vehicle().state().angle);
  std::set<std::tuple<decimal_t, decimal_t, int>> dist_set;
  if (map_->GetNearestLaneIdUsingState(state_3dof, navi_path, &ego_lane_id,
                                       &distance_to_lane,
                                       &arc_len) != kSuccess) {
    printf("[GetEgoLaneIdByPosition]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  *lane_id = ego_lane_id;
  return kSuccess;
}

// 透传最近 Lane、横向距离和投影弧长查询。
ErrorType BehaviorPlannerMapAdapter::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  std::set<std::tuple<decimal_t, decimal_t, int>> dist_set;
  if (map_->GetNearestLaneIdUsingState(state, navi_path, id, distance,
                                       arc_len) != kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// 透传车道到导航路径的拓扑可达性及换道次数。
ErrorType BehaviorPlannerMapAdapter::IsTopologicallyReachable(
    const int lane_id, const std::vector<int> &path, int *num_lane_changes,
    bool *res) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  if (map_->IsTopologicallyReachable(lane_id, path, num_lane_changes, res) !=
      kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// 从复制出的 SemanticLaneSet 查询右换道可用性和目标 ID。
ErrorType BehaviorPlannerMapAdapter::GetRightLaneId(const int lane_id,
                                                    int *r_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.r_change_avbl) {
      *r_lane_id = it->second.r_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

// 从 SemanticLaneSet 查询左换道可用性和目标 ID。
ErrorType BehaviorPlannerMapAdapter::GetLeftLaneId(const int lane_id,
                                                   int *l_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.l_change_avbl) {
      *l_lane_id = it->second.l_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

// 按 ID 复制连续 Lane，并额外拒绝无效几何。
ErrorType BehaviorPlannerMapAdapter::GetLaneByLaneId(const int lane_id,
                                                     Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
    if (!lane->IsValid()) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

// 覆盖输出为指定 Lane 的纵向后继 ID 列表。
ErrorType BehaviorPlannerMapAdapter::GetChildLaneIds(
    const int lane_id, std::vector<int> *child_ids) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // 使用 assign 覆盖调用方旧内容。
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

// 覆盖输出为指定 Lane 的纵向前驱 ID 列表。
ErrorType BehaviorPlannerMapAdapter::GetFatherLaneIds(
    const int lane_id, std::vector<int> *father_ids) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    father_ids->assign(it->second.father_id.begin(),
                       it->second.father_id.end());
  }
  return kSuccess;
}

// 透传局部参考线离散样本截取。
ErrorType BehaviorPlannerMapAdapter::GetLocalLaneSamplesByState(
    const State &state, const int lane_id, const std::vector<int> &navi_path,
    const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
    vec_Vecf<2> *samples) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLocalLaneSamplesByState(state, lane_id, navi_path,
                                       max_reflane_dist, max_backward_dist,
                                       samples) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// 透传按横向行为构造参考 Lane，并验证返回几何有效性。
ErrorType BehaviorPlannerMapAdapter::GetRefLaneForStateByBehavior(
    const State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality, Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetRefLaneForStateByBehavior(state, navi_path, behavior,
                                         max_forward_len, max_back_len,
                                         is_high_quality, lane) != kSuccess) {
    return kWrongStatus;
  }
  if (!lane->IsValid()) {
    return kWrongStatus;
  }
  return kSuccess;
}

// 透传参考 Lane 上的前车查询。
ErrorType BehaviorPlannerMapAdapter::GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLeadingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                                    leading_vehicle,
                                    distance_residual_ratio) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// 当前没有二次筛选策略，直接复制全部 surrounding_vehicles。
ErrorType BehaviorPlannerMapAdapter::GetKeyVehicles(
    common::VehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *key_vehicle_set = map_->surrounding_vehicles();
  return kSuccess;
}

// 直接复制 SemanticMapManager 已筛选/渲染的 semantic_key_vehicles。
ErrorType BehaviorPlannerMapAdapter::GetKeySemanticVehicles(
    common::SemanticVehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *key_vehicle_set = map_->semantic_key_vehicles();
  return kSuccess;
}

// 复制完整 LaneNet。
ErrorType BehaviorPlannerMapAdapter::GetWholeLaneNet(
    common::LaneNet *lane_net) {
  if (!is_valid_) return kWrongStatus;
  *lane_net = map_->whole_lane_net();
  return kSuccess;
}

// 透传两个显式车辆状态之间的碰撞检查。
ErrorType BehaviorPlannerMapAdapter::CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res) {
  if (!is_valid_) return kWrongStatus;
  if (map_->CheckCollisionUsingState(param_a, state_a, param_b, state_b, res) !=
      kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// 检查给定车辆状态与环境碰撞；baseline 忽略底层错误码并固定返回成功。
ErrorType BehaviorPlannerMapAdapter::CheckIfCollision(
    const common::VehicleParam &vehicle_param, const State &state, bool *res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

// 透传 Lane/状态相关速度限制查询。
ErrorType BehaviorPlannerMapAdapter::GetSpeedLimit(const State &state,
                                                   const Lane &lane,
                                                   decimal_t *speed_limit) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetSpeedLimit(state, lane, speed_limit) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// 从语义周车集合按 ID 读取横向预测；缺失 ID 时 unordered_map::at 会抛异常。
ErrorType BehaviorPlannerMapAdapter::GetPredictedBehavior(
    const int vehicle_id, common::LateralBehavior *lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  if (vehicle_id == kInvalidAgentId) return kWrongStatus;
  auto semantic_vehicle_set = map_->semantic_surrounding_vehicles();
  *lat_behavior =
      semantic_vehicle_set.semantic_vehicles.at(vehicle_id).lat_behavior;
  return kSuccess;
}

// 保存共享地图指针并无条件置有效，空指针也会通过 IsValid。
void BehaviorPlannerMapAdapter::set_map(
    std::shared_ptr<IntegratedMap> map_ptr) {
  map_ = map_ptr;
  is_valid_ = true;
}

}  // namespace planning
