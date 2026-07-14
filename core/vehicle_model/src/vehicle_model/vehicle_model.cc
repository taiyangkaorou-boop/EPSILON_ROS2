#include "vehicle_model/vehicle_model.h"

#include "common/math/calculations.h"

#include "odeint-v2/boost/numeric/odeint.hpp"

#include <boost/bind/placeholders.hpp> // Add

using namespace boost::placeholders;   // Add
namespace odeint = boost::numeric::odeint;

namespace simulator {

// 使用 2.5 m 默认轴距，并将 State 的默认值同步到内部数组。
VehicleModel::VehicleModel() : wheelbase_len_(2.5) { UpdateInternalState(); }

VehicleModel::VehicleModel(double wheelbase_len, double max_steering_angle)
    : wheelbase_len_(wheelbase_len), max_steering_angle_(max_steering_angle) {
  // 构造参数只保存，不在本阶段改变原有的合法性校验策略。
  UpdateInternalState();
}

VehicleModel::~VehicleModel() {}

void VehicleModel::Step(double dt) {
  // 控制量在整个积分区间保持常值，dt 同时作为终点和初始积分步长。
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  // 将积分结果写回 common::State，并把航向统一到标准角度区间。
  state_.vec_position(0) = internal_state_[0];
  state_.vec_position(1) = internal_state_[1];
  state_.angle = normalize_angle(internal_state_[2]);
  state_.steer = internal_state_[3];

  // 积分后再施加前轮机械转角上限；正负方向使用对称界限。
  if (fabs(state_.steer) >= fabs(max_steering_angle_)) {
    if (state_.steer > 0)
      state_.steer = max_steering_angle_;
    else
      state_.steer = -max_steering_angle_;
  }
  state_.velocity = internal_state_[4];

  // 由自行车模型几何关系恢复曲率，并记录本周期实际使用的纵向加速度。
  state_.curvature = tan(state_.steer) * 1.0 / wheelbase_len_;
  state_.acceleration = control_.acc_long;

  // 转角可能已被截断，因此用最终公开状态重新对齐下一周期的积分初值。
  UpdateInternalState();
}

void VehicleModel::operator()(const InternalState &x, InternalState &dxdt,
                              const double /* t */) {
  // 从 odeint 数组重建临时状态，使微分公式与 common::State 字段语义一致。
  State cur_state;
  cur_state.vec_position(0) = x[0];
  cur_state.vec_position(1) = x[1];
  cur_state.angle = x[2];
  cur_state.steer = x[3];
  cur_state.velocity = x[4];

  // 运动学自行车模型：位置沿车体航向推进，yaw_rate=v*tan(steer)/L。
  dxdt[0] = cos(cur_state.angle) * cur_state.velocity;
  dxdt[1] = sin(cur_state.angle) * cur_state.velocity;
  dxdt[2] = tan(cur_state.steer) * cur_state.velocity / wheelbase_len_;

  // 前轮转角和车速分别由常值转角速度、纵向加速度积分。
  dxdt[3] = control_.steer_rate;
  dxdt[4] = control_.acc_long;
}

void VehicleModel::set_control(const Control &control) {
  // 原始 baseline 直接接收上层控制；控制约束留待调用者或后续修复阶段处理。
  control_ = control;
  // TODO: (@denny.ding) add control limit here
}

const VehicleModel::State &VehicleModel::state(void) const { return state_; }

void VehicleModel::set_state(const VehicleModel::State &state) {
  // 外部状态一经覆盖，立即同步内部数组，避免下一次积分仍从旧值起步。
  state_ = state;
  UpdateInternalState();
}

void VehicleModel::UpdateInternalState(void) {
  // 内部索引契约：[x, y, yaw, front_wheel_steer, velocity]。
  internal_state_[0] = state_.vec_position(0);
  internal_state_[1] = state_.vec_position(1);
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.steer;
  internal_state_[4] = state_.velocity;
}

}  // namespace simulator
