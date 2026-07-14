#ifndef _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_
#define _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_

#include "common/basics/basics.h"

#include <deque>

namespace control {

/**
 * @brief 保存有限误差历史的离散标量 PID 控制器。
 *
 * 每次调用按 desired-true 计算误差，积分项遍历整个历史窗口，微分项使用最近两次误差，
 * 最后返回 P/I/D 三项直接相加的未限幅控制量。
 */
class PIDControl {
 public:
  /// PID 三项增益；默认 P=1、I=1、D=0.5。
  struct ControlParam {
    decimal_t kP;
    decimal_t kI;
    decimal_t kD;
    ControlParam() : kP(1.0), kI(1.0), kD(0.5) {}
    ControlParam(const decimal_t p, const decimal_t i, const decimal_t d)
        : kP(p), kI(i), kD(d) {}
  };

  /// 使用默认离散步长 0.05 s 和最大历史长度 1000 构造。
  PIDControl(const ControlParam& param);
  /// 使用调用方离散步长和最大历史长度 1000 构造。
  PIDControl(const ControlParam& param, const decimal_t dt);
  /// 追加本次误差，计算滑窗积分/后向差分微分，并返回未饱和 PID 输出。
  decimal_t CalculatePIDControl(const decimal_t desired_state, const decimal_t true_state);

 private:
  // 固定控制参数和离散步长。
  ControlParam param_;
  decimal_t dt_;
  // 最大历史长度及从旧到新保存的误差序列。
  int max_history_len_;
  std::deque<decimal_t> error_hist_;
};

}  // namespace control

#endif
