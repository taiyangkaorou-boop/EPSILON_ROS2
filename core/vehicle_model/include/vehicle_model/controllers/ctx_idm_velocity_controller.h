#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_

#include "common/basics/basics.h"
#include "vehicle_model/ctx_idm_model.h"

namespace control {

/**
 * @brief 对 ContextIntelligentDriverModel 的单步纵向速度控制包装。
 *
 * 除常规自车/前车纵向状态外，还输入目标位置和目标速度，模型用上下文跟踪项推进 dt；
 * 每次调用创建临时模型，不保存历史状态。
 */
class ContextIntelligentVelocityControl {
 public:
  /// 设置六维上下文状态并积分一步，将下一时刻速度截断为非负后输出。
  static ErrorType CalculateDesiredVelocity(
      const common::IntelligentDriverModel::Param& idm_param,
      const simulator::ContextIntelligentDriverModel::CtxParam& ctx_param,
      const decimal_t s, const decimal_t s_front, const decimal_t s_target,
      const decimal_t v, const decimal_t v_front, const decimal_t v_target,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif  //_CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_
