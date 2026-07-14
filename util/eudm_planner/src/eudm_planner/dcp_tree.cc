/**
 * @file dcp_tree.cc
 * @author HKUST Aerial Robotics Group
 * @brief DcpTree 候选动作脚本生成实现。
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  // 两参数构造让最后一层时长与普通层一致。
  last_layer_time_ = layer_time_;
  GenerateActionScript();
}

DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time,
                 const decimal_t& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  // 构造阶段即生成默认 ongoing action 下的全部候选。
  GenerateActionScript();
}

// ongoing action 或配置变化后，调用同一生成函数覆盖旧脚本。
ErrorType DcpTree::UpdateScript() { return GenerateActionScript(); }

std::vector<DcpTree::DcpAction> DcpTree::AppendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const {
  // 值拷贝输入序列，保证调用者原容器不被修改。
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}

ErrorType DcpTree::GenerateActionScript() {
  // 每次重建都清除上一组候选。
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  // 纵向动作在整条脚本内保持不变，分别枚举 M/A/D。
  for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) {
    ongoing_action_seq.clear();
    ongoing_action_seq.push_back(
        DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    // h 表示保持 ongoing 横向动作的层数；每个时机枚举另外两种横向动作。
    for (int h = 1; h < tree_height_; ++h) {
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT);
           lat++) {
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          // 切换后把新横向动作重复到树末端，不再二次切换。
          auto actions = AppendActionSequence(
              ongoing_action_seq,
              DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_),
              tree_height_ - h);
          action_script_.push_back(actions);
        }
      }
      ongoing_action_seq.push_back(
          DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }
    // 额外加入全时域不切换横向行为的候选。
    action_script_.push_back(ongoing_action_seq);
  }
  // 所有候选的最后一层统一覆盖为 last_layer_time_。
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }
  return kSuccess;
}

decimal_t DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  // 所有脚本设计为相同层时长，因此只累加第一条。
  decimal_t planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace planning
