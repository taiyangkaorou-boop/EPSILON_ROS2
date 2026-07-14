#include "vehicle_model/controllers/pid_controller.h"

namespace control {
PIDControl::PIDControl(const ControlParam& param)
    : param_(param), dt_(0.05), max_history_len_(1000) {
  // 日志文本标为 ms，但实际直接打印以秒保存的 0.05。
  printf("default PID controller with dt: %lf ms.\n", dt_);
}
PIDControl::PIDControl(const ControlParam& param, const decimal_t dt)
    : param_(param), dt_(dt), max_history_len_(1000) {}

decimal_t PIDControl::CalculatePIDControl(const decimal_t desired_state,
                                          const decimal_t true_state) {
  // 当前误差定义为期望状态减真实状态，并在计算前追加到历史尾部。
  decimal_t et = desired_state - true_state;
  error_hist_.push_back(et);

  // 积分项每次重新遍历全部历史误差并按固定 dt_ 做矩形积分。
  decimal_t int_e = 0.0;
  for (auto& err : error_hist_) {
    int_e += err * dt_;
  }

  // 历史至少包含三个误差时才用最近两个误差计算后向差分微分项。
  decimal_t deriv_e = 0.0;
  int num_errors = static_cast<int>(error_hist_.size());
  if (num_errors > 2) {
    deriv_e =
        (error_hist_.at(num_errors - 1) - error_hist_.at(num_errors - 2)) / dt_;
  }

  // 超出最大长度时在本次积分/微分之后删除最旧误差。
  if (num_errors > max_history_len_) error_hist_.pop_front();

  // 输出不做抗积分饱和、限幅、滤波或有限性处理。
  return param_.kP * et + param_.kI * int_e + param_.kD * deriv_e;
}

}  // namespace control
