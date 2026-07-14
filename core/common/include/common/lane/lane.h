#ifndef _CORE_COMMON_INC_COMMON_LANE_LANE_H__
#define _CORE_COMMON_INC_COMMON_LANE_LANE_H__

#include "common/basics/config.h"
#include "common/spline/spline.h"
#include "common/state/state.h"

namespace common {

/**
 * @brief 二维参考车道的连续几何表示。
 *
 * Lane 按值持有一条以参数 s 查询的五次分段多项式样条。项目通常用离散中心线的
 * 累积弦长构造 s，因此接口沿用“arc length”命名；但本类本身不重新参数化，也不
 * 保证样条导数的模恒为 1。该类只负责几何查询和点到曲线的局部投影，不保存车道
 * 拓扑、宽度、限速或交通规则。
 */
class Lane {
 public:
  /// 车道位置样条；阶数和空间维数由 config.h 中的 LaneDegree/LaneDim 固定。
  typedef Spline<LaneDegree, LaneDim> SplineType;

  /// 构造无有效样条的车道，调用几何查询前必须先设置位置样条。
  Lane() {}

  /// 以给定位置样条构造有效车道；调用方负责保证样条内容及参数域合理。
  Lane(const SplineType& position_spline)
      : position_spline_(position_spline), is_valid_(true) {}

  /// 返回位置样条是否已经成功写入，不检查其系数、参数单调性或几何质量。
  bool IsValid() const { return is_valid_; }

  /**
   * @brief 设置车道的位置样条。
   * @param position_spline 阶数和维度与 SplineType 一致的位置样条。
   *
   * 参数域为空时保持原对象不变；非空时复制样条并把车道标记为有效。
   */
  void set_position_spline(const SplineType& position_spline) {
    if (position_spline.vec_domain().empty()) return;
    position_spline_ = position_spline;
    is_valid_ = true;
  }

  /**
   * @brief 查询二维位置曲线在 s 处的有符号曲率及其导数。
   * @param arc_length 待查询的样条参数 s。
   * @param curvature 输出有符号曲率。
   * @param curvature_derivative 输出实现中使用三阶导数组合得到的曲率导数。
   * @return 参数越界、车道无效或维度不是二维时返回 kIllegalInput，否则成功。
   */
  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature,
                                    decimal_t* curvature_derivative) const;

  /**
   * @brief 查询二维位置曲线在 s 处的有符号曲率。
   * @param arc_length 待查询的样条参数 s。
   * @param curvature 输出曲率。
   */
  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature) const;

  /**
   * @brief 查询位置样条对参数 s 的 d 阶导数，d=0 等价于位置查询。
   * @param arc_length 待查询的参数 s。
   * @param d 导数阶数。
   * @param derivative 输出导数向量。
   * @return 直接透传 Spline::evaluate 的错误码。
   */
  ErrorType GetDerivativeByArcLength(const decimal_t arc_length, const int d,
                                     Vecf<LaneDim>* derivative) const;

  /// 查询 s 处的世界坐标位置，错误码直接来自底层位置样条。
  ErrorType GetPositionByArcLength(const decimal_t arc_length,
                                   Vecf<LaneDim>* derivative) const;

  /// 查询 s 处的单位切向；一阶导数过小时返回 kWrongStatus。
  ErrorType GetTangentVectorByArcLength(const decimal_t arc_length,
                                        Vecf<LaneDim>* tangent_vector) const;

  /// 查询由单位切向逆时针旋转 90 度得到的二维单位法向。
  ErrorType GetNormalVectorByArcLength(const decimal_t arc_length,
                                       Vecf<LaneDim>* normal_vector) const;

  /// 查询单位切向对应的世界航向角。
  ErrorType GetOrientationByArcLength(const decimal_t arc_length,
                                      decimal_t* angle) const;

  /**
   * @brief 将世界坐标点投影到车道并返回对应参数 s。
   *
   * 先以三个参数点执行有限次粗搜索选择初值，再调用 Newton 迭代求距离平方的局部
   * 极小值。该接口不保证得到整条曲线上的全局最近点。
   */
  ErrorType GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
                                      decimal_t* arc_length) const;

  /**
   * @brief 从指定初值开始，以 Newton 法求点到车道的局部投影参数。
   * @param vec_position 待投影的世界坐标点。
   * @param initial_guess 参数初值；实现会先截断到样条定义域。
   * @param arc_length 输出投影参数。
   */
  ErrorType GetArcLengthByVecPositionWithInitialGuess(
      const Vecf<LaneDim>& vec_position, const decimal_t& initial_guess,
      decimal_t* arc_length) const;

  /// 检查车道有效性以及 s 是否位于带 kEPS 容差的样条参数域内。
  ErrorType CheckInputArcLength(const decimal_t arc_length) const;

  /// 返回位置样条副本；调用方的修改不会回写当前 Lane。
  SplineType position_spline() const { return position_spline_; }

  /// 返回样条参数域下界。
  decimal_t begin() const { return position_spline_.begin(); }

  /// 返回样条参数域上界。
  decimal_t end() const { return position_spline_.end(); }

  /// 将底层样条内容输出到标准输出，供调试使用。
  void print() const { position_spline_.print(); }

 private:
  /// 世界位置关于参数 s 的分段多项式样条。
  SplineType position_spline_;

  /// 仅表示是否写入过非空参数域样条，不等价于几何或数值有效性验证。
  bool is_valid_ = false;
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_LANE_LANE_H__
