#ifndef _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__

#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/math/calculations.h"
#include "common/solver/qp_solver.h"
#include "common/spline/bezier.h"
#include "common/spline/lookup_table.h"
#include "common/spline/polynomial.h"
#include "common/spline/spline.h"
#include "common/state/state.h"
#include "common/state/waypoint.h"
#include "tk_spline/spline.h"

namespace common {

/**
 * @brief 固定阶数、固定维数的样条构造与 corridor 优化静态工具集。
 *
 * 该模板把离散样本、状态边界或时空走廊转换为 Polynomial Spline/BezierSpline。
 * 实现位于 .cc，当前仅显式实例化 <5,2> 与 <5,1>，其他模板参数即使头文件可见也
 * 可能在链接阶段缺少定义。
 */
template <int N_DEG, int N_DIM>
class SplineGenerator {
 public:
  /// 普通分段多项式样条输出类型。
  typedef Spline<N_DEG, N_DIM> SplineType;
  /// 时间缩放 Bezier 样条输出类型。
  typedef BezierSpline<N_DEG, N_DIM> BezierSplineType;

  /**
   * @brief 按给定参数对离散位置样本做自然三次样条插值。
   * @param samples 待插值的 N_DIM 维位置样本。
   * @param para 与样本一一对应的全局参数。
   * @param spline 输出统一 N_DEG 阶容器；三次以上系数置零。
   * @note 参数间隔大于 7 时会先在线性段上自动加密，以改善后续车道投影初值。
   */
  static ErrorType GetCubicSplineBySampleInterpolation(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      SplineType* spline);

  /**
   * @brief 以正则化最小二乘拟合分段五次样条，并施加段间位置到 jerk 连续约束。
   * @param samples 待拟合位置样本。
   * @param para 样本参数，通常为累计弦长。
   * @param breaks 样条分段断点。
   * @param regulator 最高三个多项式系数的正则权重尺度。
   * @param spline 输出普通分段样条。
   * @note 矩阵规模随样本数、分段数和维数增长，可能是高开销操作。
   */
  static ErrorType GetQuinticSplineBySampleFitting(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      const Eigen::ArrayXf& breaks, const decimal_t regulator,
      SplineType* spline);

  /**
   * @brief 把位置样本与参数逐项封装为仅固定位置的带时间戳 Waypoint。
   * @param samples N_DIM 维位置样本。
   * @param para 与样本一一对应的时间/参数戳。
   * @param waypoints 输出容器；函数会先清空。
   */
  static ErrorType GetWaypointsFromPositionSamples(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      vec_E<Waypoint<N_DIM>>* waypoints);

  /**
   * @brief 将世界坐标 State 序列逐段连接为满足两端二阶状态的五次样条。
   * @param para 与状态一一对应的分段参数。
   * @param state_vec 世界坐标车辆状态序列。
   * @param spline 输出样条；只填充前两个空间维，其余维度置零。
   */
  static ErrorType GetSplineFromStateVec(const std::vector<decimal_t>& para,
                                         const vec_E<State>& state_vec,
                                         SplineType* spline);

  /**
   * @brief 将 FreeState 序列逐段连接为满足两端位置、速度、加速度的五次样条。
   * @param para 与状态一一对应的分段参数。
   * @param free_state_vec 自由坐标状态序列。
   * @param spline 输出样条；只填充前两个空间维，其余维度置零。
   */
  static ErrorType GetSplineFromFreeStateVec(
      const std::vector<decimal_t>& para,
      const vec_E<FreeState>& free_state_vec, SplineType* spline);

  /**
   * @brief 在时空走廊内求带参考点接近项的最优时间缩放 Bezier 样条。
   * @param cubes 按时间顺序排列的 N_DIM 维时空语义走廊分段。
   * @param start_constraints 起点各阶导数约束，通常依次为位置、速度、加速度。
   * @param end_constraints 终点各阶导数约束。
   * @param ref_stamps 参考点时间戳。
   * @param ref_points 与时间戳一一对应的参考位置。
   * @param weight_proximity 参考点二次接近代价权重。
   * @param bezier_spline 输出 Bezier 样条。
   */
  static ErrorType GetBezierSplineUsingCorridor(
      const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const vec_E<Vecf<N_DIM>>& start_constraints,
      const vec_E<Vecf<N_DIM>>& end_constraints,
      const std::vector<decimal_t>& ref_stamps,
      const vec_E<Vecf<N_DIM>>& ref_points, const decimal_t& weight_proximity,
      BezierSplineType* bezier_spline);

  /**
   * @brief 在时空走廊内求只含平滑目标和边界/走廊约束的 Bezier 样条。
   * @param cubes 按时间顺序排列的时空走廊分段。
   * @param start_constraints 起点各阶导数约束。
   * @param end_constraints 终点各阶导数约束。
   * @param bezier_spline 输出 Bezier 样条。
   */
  static ErrorType GetBezierSplineUsingCorridor(
      const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const vec_E<Vecf<N_DIM>>& start_constraints,
      const vec_E<Vecf<N_DIM>>& end_constraints,
      BezierSplineType* bezier_spline);

};  // class SplineGenerator

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__
