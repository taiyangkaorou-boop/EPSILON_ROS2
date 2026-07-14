/**
 * @file state.h
 * @brief 定义车辆后轴中心处的基础二维运动状态。
 */
#ifndef _COMMON_INC_COMMON_STATE_STATE_H__
#define _COMMON_INC_COMMON_STATE_STATE_H__

#include "common/basics/basics.h"

namespace common {
/**
 * @brief 车辆后轴中心处的二维标量运动状态。
 *
 * vec_position 与 angle 给出世界位姿；velocity、acceleration 为沿车身航向的标量，
 * curvature 为轨迹曲率，steer 为转向角。几何中心位置需结合 VehicleParam::d_cr 计算。
 */
struct State {
  decimal_t time_stamp{0.0};
  Vecf<2> vec_position{Vecf<2>::Zero()};
  decimal_t angle{0.0};
  decimal_t curvature{0.0};
  decimal_t velocity{0.0};
  decimal_t acceleration{0.0};
  decimal_t steer{0.0};
  /// 输出全部状态字段，供调试和日志核对。
  void print() const {
    printf("State:\n");
    printf(" -- time_stamp: %lf.\n", time_stamp);
    printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
    printf(" -- angle: %lf.\n", angle);
    printf(" -- curvature: %lf.\n", curvature);
    printf(" -- velocity: %lf.\n", velocity);
    printf(" -- acceleration: %lf.\n", acceleration);
    printf(" -- steer: %lf.\n", steer);
  }

  /// 返回 `[x, y, yaw]` 三自由度位姿，不包含速度和曲率。
  Vec3f ToXYTheta() const {
    return Vec3f(vec_position(0), vec_position(1), angle);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_STATE_H__
