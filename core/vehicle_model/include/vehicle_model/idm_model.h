#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDM_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDM_MODEL_H__

#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "common/basics/basics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/**
 * @brief 用 Boost odeint 积分 common ACC/IDM 加速度公式的四状态纵向模型。
 *
 * 内部状态依次为自车位置/速度和前车位置/速度。前车按恒速运动，自车加速度由 common
 * IntelligentDriverModel 计算；Step 后把 odeint 数组同步回公开 State。
 */
class IntelligentDriverModel {
 public:
  using Param = common::IntelligentDriverModel::Param;
  using State = common::IntelligentDriverModel::State;

  /// 使用 Param/State 默认值构造并同步内部数组。
  IntelligentDriverModel();

  /// 使用指定 IDM 参数和默认 State 构造。
  IntelligentDriverModel(const Param &parm);

  /// 类不拥有外部资源，析构为空。
  ~IntelligentDriverModel();

  /// 返回当前公开纵向 State 的 const 引用。
  const State &state(void) const;

  /// 覆盖公开 State 并同步 odeint 内部数组。
  void set_state(const State &state);

  /// 从积分时间 0 到 dt 调用 odeint，并把结果同步回 State。
  void Step(double dt);

  // odeint 需要公开访问的四维状态和微分算子。
  typedef boost::array<double, 4> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// 把公开 State 的四个字段复制到 internal_state_。
  void UpdateInternalState(void);

  /// 预留的一步显式运动学更新；当前 Step 未调用。
  void Linear(const InternalState &x, const double dt, InternalState *x_out);

  // odeint 工作数组、IDM 参数和公开状态。
  InternalState internal_state_;

  Param param_;
  State state_;
};
}  // namespace simulator

#endif
