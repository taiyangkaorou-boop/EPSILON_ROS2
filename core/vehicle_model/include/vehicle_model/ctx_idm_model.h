#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include "common/basics/basics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/**
 * @brief 同时保存前车状态和目标纵向状态的六维上下文速度模型。
 *
 * 当前动力学实际使用目标位置/速度跟踪加速度，前车/IDM 加速度虽被计算但未融合到最终
 * acc；目标按恒速运动，模型用 odeint 积分六维状态。
 */
class ContextIntelligentDriverModel {
 public:
  using IdmParam = common::IntelligentDriverModel::Param;
  using IdmState = common::IntelligentDriverModel::State;

  /// 目标位置误差到参考速度、参考速度误差到加速度的两级增益。
  struct CtxParam {
    decimal_t k_s = 0.5;
    decimal_t k_v = 2.0 * k_s;

    CtxParam() = default;
    CtxParam(const decimal_t &_k_s, const decimal_t &_k_v)
        : k_s(_k_s), k_v(_k_v) {}
  };

  /// 自车、前车和目标的纵向位置/速度状态。
  struct CtxIdmState {
    decimal_t s{0.0};        // 自车纵向位置
    decimal_t v{0.0};        // 自车纵向速度
    decimal_t s_front{0.0};  // 前车纵向位置
    decimal_t v_front{0.0};
    decimal_t s_target{0.0};
    decimal_t v_target{0.0};
  };

  /// 使用默认 IDM/Context 参数和零状态构造。
  ContextIntelligentDriverModel();

  /// 使用指定 IDM 与 Context 参数、零状态构造。
  ContextIntelligentDriverModel(const IdmParam &idm_parm,
                                const CtxParam &ctx_param);

  /// 类不拥有外部资源，析构为空。
  ~ContextIntelligentDriverModel();

  /// 返回当前六维公开状态的 const 引用。
  const CtxIdmState &state(void) const;

  /// 覆盖公开状态并同步内部数组。
  void set_state(const CtxIdmState &state);

  /// 从积分时间 0 到 dt 调用 odeint 并同步结果。
  void Step(double dt);

  // odeint 需要公开访问的六维状态和微分算子。
  typedef boost::array<double, 6> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// 把公开六维状态复制到 internal_state_。
  void UpdateInternalState(void);

  // odeint 工作数组、IDM/Context 参数和公开状态。
  InternalState internal_state_;

  IdmParam idm_param_;
  CtxParam ctx_param_;

  CtxIdmState state_;
};
}  // namespace simulator

#endif  // _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__
