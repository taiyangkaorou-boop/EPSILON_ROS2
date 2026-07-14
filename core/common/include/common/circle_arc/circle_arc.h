/**
 * @file circle_arc.h
 * @brief 恒曲率圆弧或零曲率直线段的解析几何表示。
 * @author ZHANG Lu
 */

#ifndef _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_
#define _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"

namespace common {

/**
 * @brief 由起始平面状态、恒定曲率和有符号弧长定义的运动基元。
 *
 * 状态向量按 [x, y, theta] 解释，参数 s 沿基元弧长方向查询。曲率精确等于零时使用
 * 直线公式，否则使用圆弧公式；本类不限制 s 必须位于该基元的弧长范围内。
 */
class CircleArc {
 public:
  /// 默认构造不初始化几何字段，必须在读取前由调用方重新赋值完整对象。
  CircleArc() {}
  /// 构造恒曲率基元并解析计算圆心、圆心角和末状态。
  CircleArc(const Vec3f &start_state, const double &curvature,
            const double &arc_length);
  ~CircleArc() {}

  /// 返回有符号曲率，正值表示向左转。
  inline double curvature() const { return curvature_; }
  /// 返回有符号弧长。
  inline double arc_length() const { return arc_length_; }
  /// 返回 curvature*arc_length；直线分支未显式初始化该字段。
  inline double central_angle() const { return central_angle_; }

  /// 返回起始 [x, y, theta]。
  inline Vec3f start_state() const { return start_state_; }
  /// 返回按解析模型传播后的末状态，航向未归一化。
  inline Vec3f final_state() const { return final_state_; }
  /// 返回圆弧圆心；直线分支未显式初始化该字段。
  inline Vec2f center() const { return center_; }

  /// 返回参数 s 处的 x 坐标、关于 s 的一阶导数和二阶导数。
  double x_d0(const double s) const;
  double x_d1(const double s) const;
  double x_d2(const double s) const;

  /// 返回参数 s 处的 y 坐标、关于 s 的一阶导数和二阶导数。
  double y_d0(const double s) const;
  double y_d1(const double s) const;
  double y_d2(const double s) const;

  /// 返回参数 s 处的航向及航向对 s 的一阶导数。
  double theta_d0(const double s) const;
  double theta_d1(const double s) const;

  /// 单位切向的 x/y 分量；解析弧长参数化使其模长为 1。
  double tx_d0(const double s) const { return x_d1(s); }
  double ty_d0(const double s) const { return y_d1(s); }

  /// 左单位法向的 x/y 分量，由切向逆时针旋转 90 度得到。
  double nx_d0(const double s) const { return -this->ty_d0(s); }
  double ny_d0(const double s) const { return this->tx_d0(s); }

  /// 返回沿左法向偏移 offs 后的 x 坐标及其声明的一阶导数接口。
  double x_offs_d0(const double s, const double offs) const;
  double x_offs_d1(const double s, const double offs) const;

  /// 返回沿左法向偏移 offs 后的 y 坐标及其声明的一阶导数接口。
  double y_offs_d0(const double s, const double offs) const;
  double y_offs_d1(const double s, const double offs) const;

  /// 返回偏移曲线航向；当前实现直接沿用中心线航向。
  double theta_offs_d0(const double s, const double offs) const;

  /**
   * @brief 以固定有符号步长采样 [x, y, theta] 并追加到输出向量。
   * @note s_step 必须非零且符号与 arc_length 一致，否则循环可能无法结束。
   */
  void GetSampledStates(const double s_step,
                        std::vector<Vec3f> *p_sampled_states) const;

 private:
  /// 起点和由解析传播得到的终点状态。
  Vec3f start_state_;
  Vec3f final_state_;

  /// 恒定有符号曲率与有符号弧长。
  double curvature_;
  double arc_length_;

  /// 区分非零曲率圆弧与零曲率直线。
  bool is_arc_ = true;
  /// 圆心角，仅在非零曲率构造分支中赋值。
  double central_angle_;
  /// 圆心坐标，仅在非零曲率构造分支中赋值。
  Vec2f center_;
};  // CircleArc

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_
