#ifndef _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_BEHAVIOR_PREDICTION_H__
#define _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_BEHAVIOR_PREDICTION_H__

#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/mobil/mobil_model.h"
#include "common/rss/rss_checker.h"

namespace common {

/**
 * @brief 使用 MOBIL 加速度收益生成 LK/LCL/LCR 启发式概率分布。
 *
 * 输入车道顺序固定为 `[当前, 左, 右]`，相邻车及其 FrenetState 数组必须使用同一顺序。
 * 输出不是统计学习得到的校准概率，而是安全门控后的收益归一化分数。
 */
class MobilBehaviorPrediction {
 public:
  /**
   * @brief 计算当前/左右车道加速度变化，并输出横向行为概率分布。
   * @param nearby_vehicles 当前实现未使用的周车集合参数。
   * @param res 输出 LK/LCL/LCR 分布并置有效标记。
   */
  static ErrorType LateralBehaviorPrediction(
      const Vehicle &vehicle, const vec_E<Lane> &lanes,
      const vec_E<common::Vehicle> &leading_vehicles,
      const vec_E<common::FrenetState> &leading_frenet_states,
      const vec_E<common::Vehicle> &following_vehicles,
      const vec_E<common::FrenetState> &follow_frenet_states,
      const common::VehicleSet &nearby_vehicles, ProbDistOfLatBehaviors *res);

  /**
   * @brief 把左右安全标记及 MOBIL 收益截断映射为三类启发式概率。
   * @note 收益先从固定区间 [-1,6] 线性映射到 [0,1]。
   */
  static ErrorType RemapGainsToProb(const bool is_lcl_safe,
                                    const decimal_t mobil_gain_left,
                                    const bool is_lcr_safe,
                                    const decimal_t mobil_gain_right,
                                    ProbDistOfLatBehaviors *res);
};

}  // namespace common

#endif
