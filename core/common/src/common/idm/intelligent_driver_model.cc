#include "common/idm/intelligent_driver_model.h"

namespace common {

// 原始 IDM：自由流加速项减去期望动态间距与实际净间距的平方比。
ErrorType IntelligentDriverModel::GetIdmDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, decimal_t *acc) {
  // s_star=s0+max(0,vT+v*delta_v/(2*sqrt(a*b)))。
  decimal_t s_star =
      param.kMinimumSpacing +
      std::max(0.0,
               cur_state.v * param.kDesiredHeadwayTime +
                   cur_state.v * (cur_state.v - cur_state.v_front) /
                       (2.0 * sqrt(param.kAcceleration *
                                   param.kComfortableBrakingDeceleration)));
  // 实际净间距扣除固定车辆长度，并在零处截断；零间距随后会出现在分母。
  decimal_t s_alpha =
      std::max(0.0, cur_state.s_front - cur_state.s - param.kVehicleLength);
  *acc = param.kAcceleration *
         (1.0 - pow(cur_state.v / param.kDesiredVelocity, param.kExponent) -
          pow(s_star / s_alpha, 2));
  return kSuccess;
}

// IIDM 分别修正低于/高于期望速度时的自由流项，并限制最终加速度幅值。
ErrorType IntelligentDriverModel::GetIIdmDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, decimal_t *acc) {
  // IIDM 旨在避免原始 IDM 超过期望速度时的过强制动，并恢复期望速度附近 T 的时距含义。
  // a_free 在低速区使用 IDM 自由流项，在超速区使用舒适制动构造的平滑减速项。
  decimal_t a_free =
      cur_state.v <= param.kDesiredVelocity
          ? param.kAcceleration *
                (1 - pow(cur_state.v / param.kDesiredVelocity, param.kExponent))
          : -param.kComfortableBrakingDeceleration *
                (1 - pow(param.kDesiredVelocity / cur_state.v,
                         param.kAcceleration * param.kExponent /
                             param.kComfortableBrakingDeceleration));
  // z 是期望动态间距与实际净间距之比；零净间距会使其成为无穷或 NaN。
  decimal_t s_alpha =
      std::max(0.0, cur_state.s_front - cur_state.s - param.kVehicleLength);
  decimal_t z =
      (param.kMinimumSpacing +
       std::max(0.0,
                cur_state.v * param.kDesiredHeadwayTime +
                    cur_state.v * (cur_state.v - cur_state.v_front) /
                        (2.0 * sqrt(param.kAcceleration *
                                    param.kComfortableBrakingDeceleration)))) /
      s_alpha;
  decimal_t a_out =
      cur_state.v <= param.kDesiredVelocity
          ? (z >= 1.0
                 ? param.kAcceleration * (1 - pow(z, 2))
                 : a_free * (1 - pow(z, 2.0 * param.kAcceleration / a_free)))
          : (z >= 1.0 ? a_free + param.kAcceleration * (1 - pow(z, 2))
                      : a_free);
  // 正向不超过最大加速度，负向不超过硬制动幅值。
  a_out = std::max(std::min(param.kAcceleration, a_out),
                   -param.kHardBrakingDeceleration);
  *acc = a_out;
  return kSuccess;
}

// ACC 变体将 IIDM 与 constant-acceleration heuristic 按固定 coolness 平滑融合。
ErrorType IntelligentDriverModel::GetAccDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, decimal_t *acc) {
  decimal_t acc_iidm;
  GetIIdmDesiredAcceleration(param, cur_state, &acc_iidm);

  // 简化假设前车持续以自车舒适制动幅值减速，构造 CAH 加速度。
  decimal_t ds = std::max(0.0, cur_state.s_front - cur_state.s);
  decimal_t acc_cah =
      (cur_state.v * cur_state.v * -param.kComfortableBrakingDeceleration) /
      (cur_state.v_front * cur_state.v_front -
       2 * ds * -param.kComfortableBrakingDeceleration);

  // 固定 0.99 强烈偏向 CAH，当前不可由 Param 配置。
  decimal_t coolness = 0.99;

  // IIDM 不比 CAH 更保守时直接采用 IIDM，否则用 tanh 平滑避免突变。
  if (acc_iidm >= acc_cah) {
    *acc = acc_iidm;
  } else {
    *acc =
        (1 - coolness) * acc_iidm +
        coolness * (acc_cah - param.kComfortableBrakingDeceleration *
                                  tanh((acc_iidm - acc_cah) /
                                       -param.kComfortableBrakingDeceleration));
  }
  return kSuccess;
}

}  // namespace common
