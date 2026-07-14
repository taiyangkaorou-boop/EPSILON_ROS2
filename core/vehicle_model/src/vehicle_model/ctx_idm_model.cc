#include "vehicle_model/ctx_idm_model.h"

#include "common/math/calculations.h"
#include "odeint-v2/boost/numeric/odeint.hpp"
#include <boost/bind/placeholders.hpp> // Add

using namespace boost::placeholders;   // Add
namespace odeint = boost::numeric::odeint;
using namespace boost::placeholders;

namespace simulator {

ContextIntelligentDriverModel::ContextIntelligentDriverModel() {
  // 默认参数和零状态构造后同步六维积分数组。
  UpdateInternalState();
}

ContextIntelligentDriverModel::ContextIntelligentDriverModel(
    const IdmParam &idm_parm, const CtxParam &ctx_param)
    : idm_param_(idm_parm), ctx_param_(ctx_param) {
  // 指定参数路径仍使用 CtxIdmState 默认零值。
  UpdateInternalState();
}

ContextIntelligentDriverModel::~ContextIntelligentDriverModel() {}

void ContextIntelligentDriverModel::Step(double dt) {
  // 用 dt 同时作为积分终点和初始步长。
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  // 把积分后的六维数组复制回公开状态。
  state_.s = internal_state_[0];
  state_.v = internal_state_[1];
  state_.s_front = internal_state_[2];
  state_.v_front = internal_state_[3];
  state_.s_target = internal_state_[4];
  state_.v_target = internal_state_[5];
  // 再同步回内部数组以保持表示一致。
  UpdateInternalState();
}

void ContextIntelligentDriverModel::operator()(const InternalState &x,
                                               InternalState &dxdt,
                                               const double dt) {
  // 从 odeint 数组重建当前自车、前车和目标状态。
  CtxIdmState cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];
  cur_state.s_target = x[4];
  cur_state.v_target = x[5];

  IdmState idm_state;

  // 使用未填充的默认 IdmState 计算 acc_idm；该结果随后不会进入最终 acc。
  decimal_t acc_idm;
  common::IntelligentDriverModel::GetAccDesiredAcceleration(
      idm_param_, idm_state, &acc_idm);
  acc_idm = std::max(acc_idm, -std::min(idm_param_.kHardBrakingDeceleration,
                                        cur_state.v / dt));

  // 目标位置误差修正目标速度，再由速度误差产生跟踪加速度。
  decimal_t v_ref =
      cur_state.v_target + ctx_param_.k_s * (cur_state.s_target - cur_state.s);
  decimal_t acc_track = ctx_param_.k_v * (v_ref - cur_state.v);

  // 跟踪加速度固定截断在 [-1, 1] m/s^2。
  acc_track = std::min(std::max(acc_track, -1.0), 1.0);

  // 最终加速度只取 acc_track，忽略上方计算的 acc_idm。
  decimal_t acc = acc_track;

  dxdt[0] = cur_state.v;
  dxdt[1] = acc;
  // 前车和目标都按各自当前速度做恒速传播。
  dxdt[2] = cur_state.v_front;
  dxdt[3] = 0.0;  // assume other vehicle keep the current velocity
  dxdt[4] = cur_state.v_target;
  dxdt[5] = 0.0;  // assume constant velocity for target state
}

const ContextIntelligentDriverModel::CtxIdmState &
ContextIntelligentDriverModel::state(void) const {
  return state_;
}

void ContextIntelligentDriverModel::set_state(
    const ContextIntelligentDriverModel::CtxIdmState &state) {
  // setter 同时更新公开状态和积分数组。
  state_ = state;
  UpdateInternalState();
}

void ContextIntelligentDriverModel::UpdateInternalState(void) {
  // 数组顺序固定为 [s,v,s_front,v_front,s_target,v_target]。
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
  internal_state_[4] = state_.s_target;
  internal_state_[5] = state_.v_target;
}

}  // namespace simulator
