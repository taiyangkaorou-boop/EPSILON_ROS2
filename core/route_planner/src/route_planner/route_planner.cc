/**
 * @file route_planner.cc
 * @brief RoutePlanner 模式分派、随机 Lane 拓扑扩展和导航状态机实现。
 */

#include "route_planner/route_planner.h"

namespace planning {

// 返回通用导航规划器名称。
std::string RoutePlanner::Name() {
  return std::string("Generic route planner");
}

// 重置状态为 ready；config 字符串当前未使用。
ErrorType RoutePlanner::Init(const std::string config) {
  navi_status_ = kReadyToGo;
  return kSuccess;
}

// 按导航模式推进一次状态机，并验证最终导航 Lane。
ErrorType RoutePlanner::RunOnce() {
  switch (navi_mode_) {
    case kRandomExpansion:
      NaviLoopRandomExpansion();
      break;
    case kAssignedTarget:
      NaviLoopAssignedTarget();
      break;
    default:
      assert(false);
  }
  // 模式循环返回值被忽略，统一以 navi_lane 有效性决定本周期成功。
  if (!navi_lane_.IsValid()) {
    printf("[RP]Err - fail to output valid navi lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// ready 时生成路径，in-progress 时检查进度，finished 时按 if_restart_ 重启。
ErrorType RoutePlanner::NaviLoopRandomExpansion() {
  switch (navi_status_) {
    case kReadyToGo:
      if (GetNaviPathByRandomExpansion() != kSuccess) {
        printf("[RP]Err - fail to find navi path.\n");
        break;
      }
      navi_status_ = kInProgress;
      break;
    case kInProgress:
      if (CheckNaviProgress() != kSuccess) {
        printf("[RP]Err - InProgress but fail to check navi progress.\n");
        printf("[RP]Switching back to kReadyToGo.\n");
        navi_status_ = kReadyToGo;
      }
      break;
    case kFinished:
      printf("[RP]Finish trip.\n");
      if (if_restart_) {
        printf("[RP]Restart mission.\n");
        navi_status_ = kReadyToGo;
      }
      break;
    default:
      assert(false);
  }
  return kSuccess;
}

// 指定目标导航尚未实现，固定返回成功。
ErrorType RoutePlanner::NaviLoopAssignedTarget() { return kSuccess; }

// 从最近 Lane 开始随机选择 child，达到长度上限后拟合连续导航 Lane。
ErrorType RoutePlanner::GetNaviPathByRandomExpansion() {
  navi_path_.clear();
  navi_path_.push_back(nearest_lane_id_);
  decimal_t dist_acc = 0;
  int cur_id = nearest_lane_id_;
  // 沿 child 拓扑扩展，遇到无后继 Lane 时提前结束。
  while (dist_acc < navi_path_max_length_) {
    std::vector<int> child_ids;
    GetChildLaneIds(cur_id, &child_ids);
    int n_child = static_cast<int>(child_ids.size());
    // 避免下方除零；GetChildLaneIds 的失败返回码当前未检查。
    if (n_child == 0) break;
    // 直接按 random_device 比例映射 child；边界值可能得到 n_child。
    int rand_num = std::floor(rd_gen_() / (rd_gen_.max() / n_child));
    int rand_id = child_ids[rand_num];
    dist_acc += lane_net_.lane_set.at(rand_id).length;
    navi_path_.push_back(rand_id);
    cur_id = rand_id;
  }

  // 拼接各 Lane 中心线：首段保留点 0，后续每段从点 1 开始。
  vec_Vecf<2> raw_samples;
  for (const auto &id : navi_path_) {
    if (raw_samples.empty() &&
        (int)lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1; i < (int)lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  // 用离散中心线点拟合连续导航 Lane。
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &navi_lane_) !=
      kSuccess) {
    printf("[RP]Err - fail to fitting lane with %d samples.\n",
           static_cast<int>(raw_samples.size()));
    return kWrongStatus;
  }

  // 把自车投影到新 Lane，初始化起点/当前弧长和剩余导航长度。
  Vec2f pos(ego_state_.vec_position(0), ego_state_.vec_position(1));
  if (navi_lane_.GetArcLengthByVecPosition(pos, &navi_start_arc_length_) !=
      kSuccess) {
    printf("[RP]Err - fail to locate ego state on navi lane.\n");
    return kWrongStatus;
  }
  navi_lane_.GetArcLengthByVecPosition(pos, &navi_cur_arc_len_);

  navi_path_length_ = navi_lane_.end() - navi_start_arc_length_;
  if (navi_path_length_ < 0.0) {
    printf("[PubgLane]Err - fail to get legal navi path length.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// 检查导航几何后直接把任务置为 finished；当前没有实际进度计算。
ErrorType RoutePlanner::CheckNaviProgress() {
  if (!navi_lane_.IsValid()) return kWrongStatus;
  if (navi_path_length_ < 0.0) return kWrongStatus;

  // 原进度比例判断已被移除，任意 in-progress 周期都会完成。
  navi_status_ = kFinished;
  return kSuccess;
}

// 比较最近 Lane 与路径尾 Lane；空路径时 rbegin 解引用无保护。
bool RoutePlanner::CheckIfArriveTargetLane() {
  std::cout << "[rp] Nearest = " << nearest_lane_id_ << std::endl;
  if (nearest_lane_id_ == *navi_path_.rbegin()) {
    return true;
  } else {
    return false;
  }
}

// 从 LaneNet 复制指定 Lane 的 child ID。
ErrorType RoutePlanner::GetChildLaneIds(const int lane_id,
                                        std::vector<int> *child_ids) {
  auto it = lane_net_.lane_set.find(lane_id);
  if (it == lane_net_.lane_set.end()) {
    return kWrongStatus;
  } else {
    // assign 会覆盖调用方容器中的旧 ID。
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

}  // namespace planning
