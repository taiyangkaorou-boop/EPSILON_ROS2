#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

namespace planning {
namespace eudm {

/// 行为层换道禁用、安全性、实线约束和推荐信息的汇总。
struct LaneChangeInfo {
  // 上层策略显式禁止左右换道。
  bool forbid_lane_change_left = false;
  bool forbid_lane_change_right = false;
  // 由目标 Lane 占用关系判定的左右换道不安全标志。
  bool lane_change_left_unsafe_by_occu = false;
  bool lane_change_right_unsafe_by_occu = false;
  // 左右车道线是否为不可跨越实线。
  bool left_solid_lane = false;
  bool right_solid_lane = false;
  // 上层建议优先向左/右换道。
  bool recommend_lc_left = false;
  bool recommend_lc_right = false;
};

/// EUDM 每周期接收的用户控制权、期望速度和偏好行为任务。
struct Task {
  /// true 表示车辆处于任务控制下。
  bool is_under_ctrl = false;
  /// 用户期望速度；当前字段没有类内默认值。
  double user_desired_vel;
  /// 用户偏好行为的整数编码，默认 0。
  int user_perferred_behavior = 0;
  /// 本周期换道约束/推荐信息。
  LaneChangeInfo lc_info;
};

}  // namespace eudm
}  // namespace planning

#endif
