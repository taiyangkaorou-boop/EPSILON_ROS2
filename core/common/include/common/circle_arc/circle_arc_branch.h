/**
 * @file circle_arc_branch.h
 * @brief 从同一起点展开多条恒曲率候选基元。
 * @author ZHANG Lu
 */

#ifndef _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_
#define _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/circle_arc/circle_arc.h"

namespace common {

/**
 * @brief 一组共享起点、但曲率和长度可不同的 CircleArc 候选分支。
 *
 * 每个圆弧都从同一个 start_state_ 独立生成，circle_arc_vec_ 不是首尾相接的分段
 * 路径。该类型主要用于枚举运动基元的末状态和可视化采样。
 */
class CircleArcBranch {
 public:
  /// 默认构造函数只有声明，当前源码未提供定义。
  CircleArcBranch();

  /// 保存候选参数并立即生成所有独立圆弧；两个参数向量必须等长。
  CircleArcBranch(const Vec3f &start_state,
                  const std::vector<double> &curvature_vec,
                  const std::vector<double> length_vec)
      : start_state_(start_state),
        curvature_vec_(curvature_vec),
        length_vec_(length_vec) {
    CalculateCircleArcBranch();
  }

  ~CircleArcBranch() {}

  /// 返回共享起始状态。
  inline Vec3f start_state() const { return start_state_; }
  /// 返回候选曲率向量副本。
  inline std::vector<double> curvature_vec() const { return curvature_vec_; }
  /// 返回候选弧长向量副本。
  inline std::vector<double> length_vec() const { return length_vec_; }
  /// 返回已生成圆弧向量副本。
  inline std::vector<CircleArc> circle_arc_vec() const {
    return circle_arc_vec_;
  }

  /**
   * @brief 将每条候选圆弧的末状态追加到输出向量。
   * @param p_states 输出状态容器，不会在写入前清空。
   */
  void RetFinalStates(std::vector<Vec3f> *p_states) const;

  /**
   * @brief 以固定 0.2 m 参数步长采样每条候选圆弧并追加到输出向量。
   * @param p_states 输出状态容器，不会在写入前清空。
   */
  void RetAllSampledStates(std::vector<Vec3f> *p_states) const;

 private:
  /**
   * @brief 按曲率/长度一一配对，从共享起点构造所有候选圆弧。
   */
  void CalculateCircleArcBranch();

  /// 所有候选基元共享的起点。
  Vec3f start_state_;
  /// 每个候选的恒定曲率。
  std::vector<double> curvature_vec_;
  /// 每个候选的有符号弧长。
  std::vector<double> length_vec_;
  /// 与输入参数同序的独立圆弧候选。
  std::vector<CircleArc> circle_arc_vec_;
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_
