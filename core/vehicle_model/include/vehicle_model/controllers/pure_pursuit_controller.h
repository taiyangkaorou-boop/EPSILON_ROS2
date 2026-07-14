#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_PURE_PURSUIT_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_PURE_PURSUIT_H_

#include "common/basics/basics.h"

namespace control {
/**
 * @brief 基于轴距、目标方向误差和前视距离的 Pure Pursuit 转角计算器。
 *
 * 使用 Snider 等文献中的几何关系 atan2(2L sin(alpha), lookahead)；调用方通常让前视距离
 * 随当前速度变化。本类无内部状态。
 */
class PurePursuitControl {
 public:
  /// 计算前轮期望转角并写入输出；当前不校验轴距、前视距离或角度范围。
  static ErrorType CalculateDesiredSteer(const decimal_t wheelbase_len,
                                         const decimal_t angle_diff,
                                         const decimal_t look_ahead_dist,
                                         decimal_t *steer);
};

}  // namespace control

#endif
