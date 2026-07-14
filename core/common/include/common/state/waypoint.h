/**
 * @file waypoint.h
 * @brief 定义轨迹生成器使用的 N 维位置及高阶导数约束点。
 */
#ifndef _COMMON_INC_COMMON_STATE_WAYPOINT_H__
#define _COMMON_INC_COMMON_STATE_WAYPOINT_H__

#include "common/basics/basics.h"
namespace common {

/**
 * @brief N 维轨迹边界/中间约束点。
 *
 * pos/vel/acc/jrk 分别保存位置到 jerk 的导数值；fix_* 标记相应导数是否作为硬约束，
 * stamped 表示时间 t 是否有效。结构本身不检查约束之间的一致性。
 */
template <int N_DIM>
struct Waypoint {
  Vecf<N_DIM> pos;
  Vecf<N_DIM> vel;
  Vecf<N_DIM> acc;
  Vecf<N_DIM> jrk;
  decimal_t t{0.0};
  bool fix_pos = false;
  bool fix_vel = false;
  bool fix_acc = false;
  bool fix_jrk = false;
  bool stamped = false;
};

/// 二维轨迹 waypoint。
typedef Waypoint<2> Waypoint2D;
/// 三维轨迹 waypoint。
typedef Waypoint<3> Waypoint3D;
}  // namespace common

#endif
