#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_

#include "common/basics/basics.h"

#include "vehicle_model/idm_model.h"

namespace control {

/**
 * @brief 用 simulator::IntelligentDriverModel 向前积分一步的纵向速度控制包装。
 *
 * 输入当前/前车纵向位置和速度，模型内部使用 ACC 加速度公式推进 dt，输出下一时刻非负
 * 速度；本类不保存跨调用状态。
 */
class IntelligentVelocityControl {
 public:
  /// 构造临时 IDM、设置状态并 Step(dt)，把下一速度截断到非负后写入输出。
  static ErrorType CalculateDesiredVelocity(
      const simulator::IntelligentDriverModel::Param& param, const decimal_t s,
      const decimal_t s_front, const decimal_t v, const decimal_t v_front,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif
