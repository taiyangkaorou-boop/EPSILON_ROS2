#include "vehicle_model/ideal_steer_model.h"

#include "common/math/calculations.h"

#include "odeint-v2/boost/numeric/odeint.hpp"

#include <boost/bind/placeholders.hpp> // Add

using namespace boost::placeholders;   // Add
namespace odeint = boost::numeric::odeint;

namespace simulator {

IdealSteerModel::IdealSteerModel(double wheelbase_len, double max_lon_acc,
                                 double max_lon_dec, double max_lon_acc_jerk,
                                 double max_lon_dec_jerk, double max_lat_acc,
                                 double max_lat_jerk, double max_steering_angle,
                                 double max_steer_rate, double max_curvature)
    : wheelbase_len_(wheelbase_len),
      max_lon_acc_(max_lon_acc),
      max_lon_dec_(max_lon_dec),
      max_lon_acc_jerk_(max_lon_acc_jerk),
      max_lon_dec_jerk_(max_lon_dec_jerk),
      max_lat_acc_(max_lat_acc),
      max_lat_jerk_(max_lat_jerk),
      max_steering_angle_(max_steering_angle),
      max_steer_rate_(max_steer_rate),
      max_curvature_(max_curvature) {
  // 以下调试输出保留为原始 baseline 记录，默认不参与运行。
  // printf("[DEBUG]max_lon_acc_ %lf.\n", max_lon_acc_);
  // printf("[DEBUG]max_lon_dec_ %lf.\n", max_lon_dec_);
  // printf("[DEBUG]max_lon_acc_jerk_ %lf.\n", max_lon_acc_jerk_);
  // printf("[DEBUG]max_lon_dec_jerk_ %lf.\n", max_lon_dec_jerk_);
  // printf("[DEBUG]max_lat_acc_ %lf.\n", max_lat_acc_);
  // printf("[DEBUG]max_lat_jerk_ %lf.\n", max_lat_jerk_);
  // printf("[DEBUG]max_steering_angle_ %lf.\n", max_steering_angle_);
  // printf("[DEBUG]max_steer_rate_ %lf.\n", max_steer_rate_);
  // printf("[DEBUG]max_curvature_ %lf.\n", max_curvature_);
  // 以 common::State 的默认值建立 odeint 初始数组。
  UpdateInternalState();
}

IdealSteerModel::~IdealSteerModel() {}

void IdealSteerModel::TruncateControl(const decimal_t &dt) {
  // 原始最大曲率限速逻辑目前被禁用，保留代码用于说明参数的设计意图。
  // decimal_t max_velocity_by_model =
  //     sqrt(max_lat_acc_ / std::min(fabs(state_.curvature), max_curvature_));
  // control_.velocity =
  //     std::min(std::max(0.0, control_.velocity), max_velocity_by_model);

  // 由目标速度反推纵向加速度，再限制纵向 jerk 与最终加/减速度。
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  decimal_t desired_lon_jerk = (desired_lon_acc_ - state_.acceleration) / dt;
  desired_lon_jerk =
      truncate(desired_lon_jerk, -max_lon_dec_jerk_, max_lon_acc_jerk_);
  desired_lon_acc_ = desired_lon_jerk * dt + state_.acceleration;
  desired_lon_acc_ = truncate(desired_lon_acc_, -max_lon_dec_, max_lon_acc_);

  // 用受约束的纵向加速度重算本周期可达目标速度，并禁止负速度。
  control_.velocity = std::max(state_.velocity + desired_lon_acc_ * dt, 0.0);

  // 由目标速度/转角计算期望横向加速度，并以当前曲率恢复上一时刻横向加速度。
  desired_lat_acc_ =
      pow(control_.velocity, 2) * (tan(control_.steer) / wheelbase_len_);
  decimal_t lat_acc_ori = pow(state_.velocity, 2) * state_.curvature;

  // 先限制横向 jerk，再限制最终横向加速度。
  decimal_t lat_jerk_desired = (desired_lat_acc_ - lat_acc_ori) / dt;
  lat_jerk_desired = truncate(lat_jerk_desired, -max_lat_jerk_, max_lat_jerk_);
  desired_lat_acc_ = lat_jerk_desired * dt + lat_acc_ori;
  desired_lat_acc_ = truncate(desired_lat_acc_, -max_lat_acc_, max_lat_acc_);

  // 将可达横向加速度反解为前轮转角；低速时使用固定极小分母避免直接除零。
  control_.steer = atan(desired_lat_acc_ * wheelbase_len_ /
                        std::max(pow(control_.velocity, 2), 0.1 * kBigEPS));

  // 最后限制一个周期内的转角变化率，并得到实际可达目标转角。
  desired_steer_rate_ = normalize_angle(control_.steer - state_.steer) / dt;
  desired_steer_rate_ =
      truncate(desired_steer_rate_, -max_steer_rate_, max_steer_rate_);
  control_.steer = normalize_angle(state_.steer + desired_steer_rate_ * dt);
}

void IdealSteerModel::Step(double dt) {
  // common::State 以曲率为上层几何状态，积分前先恢复对应的前轮转角。
  state_.steer = atan(state_.curvature * wheelbase_len_);
  UpdateInternalState();

  // 先施加显式目标速度/机械转角边界，再执行 jerk 与横向动力学约束。
  control_.velocity = std::max(0.0, control_.velocity);
  control_.steer =
      truncate(control_.steer, -max_steering_angle_, max_steering_angle_);
  TruncateControl(dt);

  // 由 TruncateControl 修改后的可达目标重新计算本周期常值导数。
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  desired_steer_rate_ = normalize_angle(control_.steer - state_.steer) / dt;

  // 在整个积分区间保持期望纵向加速度和前轮转角速度不变。
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  // 将五维积分结果写回公开状态，并规范化航向与前轮转角。
  state_.vec_position(0) = internal_state_[0];
  state_.vec_position(1) = internal_state_[1];
  state_.angle = normalize_angle(internal_state_[2]);
  state_.velocity = internal_state_[3];
  state_.steer = normalize_angle(internal_state_[4]);

  // 用最终转角恢复几何曲率，并记录本周期实际纵向加速度。
  state_.curvature = tan(state_.steer) * 1.0 / wheelbase_len_;
  state_.acceleration = desired_lon_acc_;

  // 对齐下一周期 odeint 初始数组；模型本身不推进 State.time_stamp。
  UpdateInternalState();
}

void IdealSteerModel::operator()(const InternalState &x, InternalState &dxdt,
                                 const double /* t */) {
  // 从 odeint 数组重建临时状态，索引顺序与普通 VehicleModel 不同。
  State cur_state;
  cur_state.vec_position(0) = x[0];
  cur_state.vec_position(1) = x[1];
  cur_state.angle = x[2];
  cur_state.velocity = x[3];
  cur_state.steer = x[4];

  // 运动学自行车几何负责平面位姿演化。
  dxdt[0] = cos(cur_state.angle) * cur_state.velocity;
  dxdt[1] = sin(cur_state.angle) * cur_state.velocity;
  dxdt[2] = tan(cur_state.steer) * cur_state.velocity / wheelbase_len_;

  // 速度和转角分别按本周期受约束的常值导数演化。
  dxdt[3] = desired_lon_acc_;
  dxdt[4] = desired_steer_rate_;
}

void IdealSteerModel::set_control(const Control &control) {
  // 这里只缓存上层目标；所有物理约束集中在 Step/TruncateControl 中处理。
  control_ = control;
}

const IdealSteerModel::State &IdealSteerModel::state(void) const {
  return state_;
}

void IdealSteerModel::set_state(const IdealSteerModel::State &state) {
  // 外部状态一经覆盖，立即同步内部数组，避免下一次积分仍从旧值起步。
  state_ = state;
  UpdateInternalState();
}

void IdealSteerModel::UpdateInternalState(void) {
  // 内部索引契约：[x, y, yaw, velocity, front_wheel_steer]。
  internal_state_[0] = state_.vec_position(0);
  internal_state_[1] = state_.vec_position(1);
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.velocity;
  internal_state_[4] = state_.steer;
}

}  // namespace simulator
