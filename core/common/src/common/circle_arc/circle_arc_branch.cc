#include "common/circle_arc/circle_arc_branch.h"

namespace common {

// 保持 circle_arc_vec_ 的顺序，将各候选末状态追加到调用方容器。
void CircleArcBranch::RetFinalStates(std::vector<Vec3f> *p_states) const {
  for (const auto &arc : circle_arc_vec_) {
    p_states->emplace_back(arc.final_state());
  }
}

// 每条候选独立采样；不同候选均会重复写入共享起点附近的样本。
void CircleArcBranch::RetAllSampledStates(std::vector<Vec3f> *p_states) const {
  for (const auto &arc : circle_arc_vec_) {
    std::vector<Vec3f> samples;
    arc.GetSampledStates(0.2, &samples);
    for (const auto &state : samples) {
      p_states->emplace_back(state);
    }
  }
}

// 曲率和长度按下标一一配对；每条弧都从 start_state_ 出发而非连接前一条弧。
void CircleArcBranch::CalculateCircleArcBranch() {
  int n_arcs = curvature_vec_.size();
  if ((int)length_vec_.size() != n_arcs) {
    // 输入尺寸不一致被视为程序错误，通过 assert 终止调试构建。
    std::cerr << "[CircleArcBranch] ERROR - Size of vec are not equal"
              << std::endl;
    assert(false);
  }
  for (int i = 0; i < n_arcs; ++i) {
    CircleArc arc(start_state_, curvature_vec_[i], length_vec_[i]);
    circle_arc_vec_.emplace_back(arc);
  }
}

}  // namespace common
