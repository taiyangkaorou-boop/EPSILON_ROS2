#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_VEHICLE_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_VEHICLE_MODEL_H__

#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "common/basics/basics.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/**
 * @brief 以转角速度和纵向加速度为输入的五状态运动学自行车模型。
 *
 * 对外状态使用 common::State；供 odeint 积分的内部状态依次为
 * `[x, y, yaw, front_wheel_steer, velocity]`。模型不负责控制限幅或时间戳推进，
 * Step 只完成连续运动学积分、航向归一化、前轮转角截断和曲率回写。
 */
class VehicleModel {
 public:
  using State = common::State;

  /// 一个积分周期内保持不变的低层控制量。
  struct Control {
    double steer_rate{0.0};  ///< 前轮转角速度，单位 rad/s。
    double acc_long{0.0};    ///< 车体纵向加速度，单位 m/s^2。

    /// 构造零转角速度、零纵向加速度控制。
    Control() {}

    /// 使用给定前轮转角速度和纵向加速度构造控制量。
    Control(const double s, const double a) : steer_rate(s), acc_long(a) {}
  };

  /// 使用默认轴距构造模型，并把默认 State 同步到 odeint 数组。
  VehicleModel();

  /// 使用指定轴距和最大前轮转角构造模型。
  VehicleModel(double wheelbase_len, double max_steering_angle);

  /// 类不拥有外部资源，析构为空。
  ~VehicleModel();

  /// 返回最近一次设置或积分后的公开车辆状态。
  const State &state(void) const;

  /// 覆盖公开状态，并同步五维 odeint 内部数组。
  void set_state(const State &state);

  /// 保存下一次 Step 使用的控制量；当前接口本身不做限幅。
  void set_control(const Control &control);

  /// 从积分时间 0 推进到 dt，并把结果同步回公开状态。
  void Step(double dt);

  // odeint 需要公开访问的五维状态类型和微分算子。
  typedef boost::array<double, 5> InternalState;

  /// 根据运动学自行车方程计算五维状态导数；积分时间参数未使用。
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// 把公开 State 中参与积分的字段复制到 internal_state_。
  void UpdateInternalState(void);

  // 公开状态、当前控制与 odeint 工作数组。
  State state_;
  Control control_;
  InternalState internal_state_;

  // 车辆几何参数；最大转角仅在 Step 积分完成后应用。
  double wheelbase_len_;
  double max_steering_angle_;
};
}  // namespace simulator

#endif
