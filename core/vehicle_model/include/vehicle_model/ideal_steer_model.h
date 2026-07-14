#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDEAL_STEER_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDEAL_STEER_MODEL_H__

#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include "common/basics/basics.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/**
 * @brief 将目标转角/目标速度转成受动力学约束状态演化的理想转向模型。
 *
 * 上层给出期望前轮转角和车速，TruncateControl 依次施加纵向加速度与 jerk、
 * 非负速度、横向加速度与 jerk、前轮转角速度约束。随后 odeint 对
 * `[x, y, yaw, velocity, front_wheel_steer]` 五维状态积分。
 */
class IdealSteerModel {
 public:
  using State = common::State;

  /// 上层规划器给出的目标前轮转角和目标车速。
  struct Control {
    double steer{0.0};     ///< 期望前轮转角，单位 rad。
    double velocity{0.0};  ///< 期望车体纵向速度，单位 m/s。

    /// 构造零转角、零目标速度控制。
    Control() {}

    /// 使用给定目标转角和目标车速构造控制量。
    Control(const double s, const double v) : steer(s), velocity(v) {}
  };

  /// 保存车辆几何、纵横向舒适性和转向约束参数，并同步默认状态。
  IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec,
                  double max_lon_acc_jerk, double max_lon_dec_jerk,
                  double max_lat_acc, double max_lat_jerk,
                  double max_steering_angle, double max_steer_rate,
                  double max_curvature);

  /// 类不拥有外部资源，析构为空。
  ~IdealSteerModel();

  /// 返回最近一次设置或积分后的公开车辆状态。
  const State &state(void) const;

  /// 覆盖公开状态，并同步五维 odeint 内部数组。
  void set_state(const State &state);

  /// 保存上层期望转角和期望速度，约束在 Step 内统一施加。
  void set_control(const Control &control);

  /// 约束控制、推进 dt 时长，并回写位置、姿态、速度、转角和加速度。
  void Step(double dt);

  /// 按纵向、横向和转向约束顺序修改当前 control_。
  void TruncateControl(const decimal_t& dt);

  // odeint 需要公开访问的五维状态类型和微分算子。
  typedef boost::array<double, 5> InternalState;

  /// 根据运动学自行车方程和本周期期望加速度/转角速度计算状态导数。
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// 把公开 State 中参与积分的字段复制到 internal_state_。
  void UpdateInternalState(void);

  // 公开状态、上层目标控制与本周期 odeint 工作数组。
  State state_;
  Control control_;

  // 经舒适性/动力学约束后，供微分算子使用的实际控制导数。
  decimal_t desired_steer_rate_;
  decimal_t desired_lon_acc_;
  decimal_t desired_lat_acc_;
  InternalState internal_state_;

  // 车辆几何及纵向、横向、转向约束参数。
  double wheelbase_len_;
  double max_lon_acc_;
  double max_lon_dec_;
  double max_lon_acc_jerk_;
  double max_lon_dec_jerk_;
  double max_lat_acc_;
  double max_lat_jerk_;
  double max_steering_angle_;
  double max_steer_rate_;
  // 当前 baseline 仅保存该参数，相关最大曲率限速代码仍被注释。
  double max_curvature_;
};
}  // namespace simulator

#endif
