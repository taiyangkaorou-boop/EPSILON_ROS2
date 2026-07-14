#include "vehicle_model/controllers/pure_pursuit_controller.h"

namespace control {
ErrorType PurePursuitControl::CalculateDesiredSteer(
    const decimal_t wheelbase_len, const decimal_t angle_diff,
    const decimal_t look_ahead_dist, decimal_t *steer) {
  // atan2 在零/负前视距离下仍会返回角度，函数不会把该输入视为错误。
  *steer = atan2(2.0 * wheelbase_len * sin(angle_diff), look_ahead_dist);
  return kSuccess;
}
}  // namespace control
