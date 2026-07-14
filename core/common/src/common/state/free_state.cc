/**
 * @file free_state.cc
 * @brief 实现标量车体状态与世界坐标速度/加速度向量之间的转换。
 */
#include "common/state/free_state.h"

#include <math.h>
namespace common {

void GetFreeStateFromState(const State& state, FreeState* free_state) {
  // 速度沿航向切向展开到世界 x/y 分量。
  free_state->position = state.vec_position;
  decimal_t cn = cos(state.angle);
  decimal_t sn = sin(state.angle);
  free_state->velocity[0] = state.velocity * cn;
  free_state->velocity[1] = state.velocity * sn;
  // 总加速度由切向加速度和 v^2*kappa 法向加速度合成。
  decimal_t normal_acc = state.velocity * state.velocity * state.curvature;
  free_state->acceleration[0] = state.acceleration * cn - normal_acc * sn;
  free_state->acceleration[1] = state.acceleration * sn + normal_acc * cn;
  free_state->angle = state.angle;
  free_state->time_stamp = state.time_stamp;
}

void GetStateFromFreeState(const FreeState& free_state, State* state) {
  // 航向由输入显式给定，位置直接复制；速度取模会丢失前进/倒车符号。
  state->angle = free_state.angle;
  state->vec_position = free_state.position;
  state->velocity = free_state.velocity.norm();
  decimal_t cn = cos(state->angle);
  decimal_t sn = sin(state->angle);
  // 将世界加速度投影到航向切向和左法向。
  Vecf<2> tangent_vec{Vecf<2>(cn, sn)};
  Vecf<2> normal_vec{Vecf<2>(-sn, cn)};
  auto a_tangent = free_state.acceleration.dot(tangent_vec);
  auto a_normal = free_state.acceleration.dot(normal_vec);
  state->acceleration = a_tangent;
  // 非低速时由 a_normal=v^2*kappa 反求曲率；低速时避免除零并置零。
  if (fabs(state->velocity) > kBigEPS) {
    state->curvature = a_normal / pow(state->velocity, 2);
  } else {
    state->curvature = 0.0;
  }
  state->time_stamp = free_state.time_stamp;
}

}  // namespace common
