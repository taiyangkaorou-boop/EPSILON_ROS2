#include "common/lane/lane_generator.h"
#include "common/basics/config.h"
#include "common/spline/spline_generator.h"

namespace common {

// 使用调用方参数化执行自然三次样条插值，并将结果封装为统一的 Lane。
ErrorType LaneGenerator::GetLaneBySampleInterpolation(
    const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
    Lane* lane) {
  // LaneDegree 当前为 5；底层生成器把三次系数写入五次多项式容器的对应位置。
  Spline<LaneDegree, LaneDim> spline;
  SplineGenerator<LaneDegree, LaneDim> spline_generator;

  if (spline_generator.GetCubicSplineBySampleInterpolation(
          samples, para, &spline) != kSuccess) {
    return kWrongStatus;
  }
  // set_position_spline 只会拒绝空参数域，因此继续检查 Lane 的有效标记。
  lane->set_position_spline(spline);
  if (!lane->IsValid()) return kWrongStatus;
  return kSuccess;
}

ErrorType LaneGenerator::GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples,
                                               Lane* lane) {
  // 以第一点为 s=0，使用相邻点的二维弦长逐段累加近似弧长参数。
  std::vector<decimal_t> para;
  double d = 0;
  para.push_back(d);
  for (int i = 1; i < (int)samples.size(); ++i) {
    double dx = samples[i](0) - samples[i - 1](0);
    double dy = samples[i](1) - samples[i - 1](1);
    d += std::hypot(dx, dy);
    para.push_back(d);
  }
  // 参数化完成后复用统一的插值入口，错误时不产生额外诊断信息。
  if (common::LaneGenerator::GetLaneBySampleInterpolation(samples, para,
                                                          lane) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType LaneGenerator::GetLaneBySampleFitting(
    const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
    const Eigen::ArrayXf& breaks, const decimal_t regulator, Lane* lane) {
  // 具体 QP/最小二乘矩阵、最高到 jerk 的段间连续性约束和正则项由
  // SplineGenerator 组装；本层只负责错误转换和 Lane 输出。
  Spline<LaneDegree, LaneDim> spline;
  SplineGenerator<LaneDegree, LaneDim> spline_generator;

  if (spline_generator.GetQuinticSplineBySampleFitting(
          samples, para, breaks, regulator, &spline) != kSuccess) {
    return kWrongStatus;
  }
  lane->set_position_spline(spline);
  if (!lane->IsValid()) return kWrongStatus;
  return kSuccess;
}

}  // namespace common
