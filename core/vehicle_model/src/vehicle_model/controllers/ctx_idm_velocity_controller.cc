#include "vehicle_model/controllers/ctx_idm_velocity_controller.h"

namespace control {

ErrorType ContextIntelligentVelocityControl::CalculateDesiredVelocity(
    const simulator::ContextIntelligentDriverModel::IdmParam& idm_param,
    const simulator::ContextIntelligentDriverModel::CtxParam& ctx_param,
    const decimal_t s, const decimal_t s_front, const decimal_t s_target,
    const decimal_t v, const decimal_t v_front, const decimal_t v_target,
    const decimal_t dt, decimal_t* velocity_at_dt) {
  // 每次调用从参数创建独立 Context-IDM 模型。
  simulator::ContextIntelligentDriverModel model(idm_param, ctx_param);
  simulator::ContextIntelligentDriverModel::CtxIdmState state;
  // 写入自车、前车和目标的纵向位置/速度；仅自车速度先截断为非负。
  state.s = s;
  state.s_front = s_front;
  state.s_target = s_target;
  state.v = std::max(0.0, v);
  state.v_front = v_front;
  state.v_target = v_target;

  model.set_state(state);
  // 通过 odeint 推进一个 dt。
  model.Step(dt);

  auto desired_state = model.state();
  // 输出下一状态速度并再次截断负值。
  *velocity_at_dt = std::max(0.0, desired_state.v);

  return kSuccess;
}

}  // namespace control
