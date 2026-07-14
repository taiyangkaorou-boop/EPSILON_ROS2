#ifndef _CORE_COMMON_INC_COMMON_LANE_LANE_GENERATOR_H__
#define _CORE_COMMON_INC_COMMON_LANE_LANE_GENERATOR_H__

#include "common/lane/lane.h"

#include "common/basics/basics.h"
#include "common/basics/config.h"

namespace common {

/**
 * @brief 从离散二维中心线样本构造 Lane 的无状态工具类。
 *
 * 本类负责选择样条生成方式并把结果写入 Lane，不保存地图拓扑，也不验证样本是否
 * 自交、参数是否严格递增或输出曲线是否满足道路几何约束。
 */
class LaneGenerator {
 public:
  /**
   * @brief 按调用方给定参数对样本点做自然三次样条插值。
   * @param samples 按行驶方向排列的二维位置样本。
   * @param para 与样本一一对应的参数，通常取累计弦长。
   * @param lane 输出车道。
   *
   * 三次多项式系数会封装进项目统一的五次 Spline 表示；稀疏参数段的加密逻辑由
   * SplineGenerator 负责。
   */
  static ErrorType GetLaneBySampleInterpolation(
      const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
      Lane* lane);

  /**
   * @brief 根据相邻样本的二维欧氏距离累计参数，再进行三次样条插值。
   * @param samples 按顺序排列的中心线位置样本。
   * @param lane 输出车道。
   */
  static ErrorType GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples,
                                         Lane* lane);

  /**
   * @brief 使用带连续性等式约束和正则项的分段五次样条最小二乘拟合车道。
   * @param samples 待拟合的位置样本。
   * @param para 与样本对应的参数，通常取累计弦长。
   * @param breaks 各样条分段的参数断点。
   * @param regulator 高阶系数正则权重；原实现建议范围为 [1e6, 1e8)。
   * @param lane 输出车道。
   */
  static ErrorType GetLaneBySampleFitting(const vec_Vecf<LaneDim>& samples,
                                          const std::vector<decimal_t>& para,
                                          const Eigen::ArrayXf& breaks,
                                          const decimal_t regulator,
                                          Lane* lane);
};

}  // namespace common

#endif
