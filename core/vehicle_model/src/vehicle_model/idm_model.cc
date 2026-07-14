#include "vehicle_model/idm_model.h"

#include "common/math/calculations.h"
#include "odeint-v2/boost/numeric/odeint.hpp"

#include <boost/bind/placeholders.hpp> // Add

using namespace boost::placeholders;   // Add
namespace odeint = boost::numeric::odeint;

namespace simulator {

// 默认 Param/State 构造后，把公开状态同步到 odeint 数组。
IntelligentDriverModel::IntelligentDriverModel() { UpdateInternalState(); }

IntelligentDriverModel::IntelligentDriverModel(const Param &parm)
    : param_(parm) {
  // 指定参数路径仍使用 State 默认值。
  UpdateInternalState();
}

IntelligentDriverModel::~IntelligentDriverModel() {}

void IntelligentDriverModel::Step(double dt) {
  // 用 dt 同时作为积分终点和初始步长；系统算子的第三参数实际是积分时间 t。
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  // Linear(internal_state_, dt, &internal_state_);

  // printf("[Internal]%lf, %lf, %lf, %lf\n", internal_state_[0],
  //        internal_state_[1], internal_state_[2], internal_state_[3]);

  // 把积分后的四维数组复制回公开 State。
  state_.s = internal_state_[0];
  state_.v = internal_state_[1];
  state_.s_front = internal_state_[2];
  state_.v_front = internal_state_[3];
  // 再把同一组 State 值写回 internal_state_，保持两套表示一致。
  UpdateInternalState();
}

void IntelligentDriverModel::Linear(const InternalState &x, const double dt,
                                    InternalState *x_out) {
  // 从内部数组重建临时 IDM State，并使用 IIDM 公式计算期望加速度。
  State cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];

  decimal_t acc;
  common::IntelligentDriverModel::GetIIdmDesiredAcceleration(param_, cur_state,
                                                             &acc);

  std::cout << "acc = " << acc << std::endl;
  // 制动下界限制为 hard braking 与恰好在 dt 内停下所需减速度中的较小者。
  acc = std::max(acc,
                 -std::min(param_.kHardBrakingDeceleration, cur_state.v / dt));

  std::cout << "acc = " << acc << std::endl;

  // 自车用匀加速公式，前车保持恒速推进。
  (*x_out)[0] = x[0] + cur_state.v * dt + 0.5 * acc * dt * dt;
  (*x_out)[1] = cur_state.v + acc * dt;
  (*x_out)[2] = x[2] + x[3] * dt;
  (*x_out)[3] = x[3];
}

void IntelligentDriverModel::operator()(const InternalState &x,
                                        InternalState &dxdt, const double dt) {
  // odeint 微分回调从当前数组重建临时 State。
  State cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];

  decimal_t acc;
  common::IntelligentDriverModel::GetAccDesiredAcceleration(param_, cur_state,
                                                            &acc);
  // 第三个形参由 odeint 传入积分时间 t，但当前实现把它当作步长用于 v/t 制动限制。
  acc = std::max(acc,
                 -std::min(param_.kHardBrakingDeceleration, cur_state.v / dt));
  dxdt[0] = cur_state.v;
  dxdt[1] = acc;
  // 前车位置按当前速度变化，前车速度导数固定为零。
  dxdt[2] = cur_state.v_front;
  dxdt[3] = 0.0;  // assume other vehicle keep the current velocity
}

const IntelligentDriverModel::State &IntelligentDriverModel::state(void) const {
  return state_;
}

void IntelligentDriverModel::set_state(
    const IntelligentDriverModel::State &state) {
  // setter 同时刷新公开状态和积分数组。
  state_ = state;
  UpdateInternalState();
}

void IntelligentDriverModel::UpdateInternalState(void) {
  // 数组顺序固定为 [s, v, s_front, v_front]。
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
}

}  // namespace simulator
