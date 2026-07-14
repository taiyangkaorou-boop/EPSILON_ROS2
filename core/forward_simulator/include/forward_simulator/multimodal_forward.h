#ifndef _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__
#define _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"

#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/**
 * @brief 将离散激进程度等级映射为 OnLaneForwardSimulation 驾驶参数。
 *
 * 当前只提供 1--5 五档静态查表，不执行多模态 rollout 或概率融合；类名表达的是预留
 * 职责。调用方必须先提供已初始化 Param，本函数只覆盖部分 IDM 字段和转向增益。
 */
class MultiModalForward {
 public:
  using Lane = common::Lane;
  using VehicleSet = common::VehicleSet;
  using GridMap = common::GridMapND<uint8_t, 2>;
  using State = common::State;
  /// 离散驾驶风格等级，合法取值为 1（保守）到 5（激进）。
  typedef int AggressivenessLevel;

  /**
   * @brief 按激进程度覆盖时距、最小间距、加速/舒适制动和转向增益。
   * @param agg_level 1--5 离散等级。
   * @param param 待修改仿真参数；其他字段保持调用前值。
   * @return 当前实现固定返回 kSuccess，非法等级仅触发 assert。
   */
  static ErrorType ParamLookUp(const AggressivenessLevel& agg_level,
                               OnLaneForwardSimulation::Param* param) {
    switch (agg_level) {
      case 1:
        // 最保守：最长时距、较大间距、最低加速和舒适制动。
        param->idm_param.kDesiredHeadwayTime = 2.0;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.0;
        param->steer_control_gain = 2.0;
        break;
      case 2:
        param->idm_param.kDesiredHeadwayTime = 1.7;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.67;
        param->steer_control_gain = 2.0;
        break;
      case 3:
        param->idm_param.kDesiredHeadwayTime = 1.5;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 4:
        param->idm_param.kDesiredHeadwayTime = 1.0;
        param->idm_param.kMinimumSpacing = 1.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 5:
        // 最激进：最短时距和最小静止间距；转向增益仍与其他等级相同。
        param->idm_param.kDesiredHeadwayTime = 0.5;
        param->idm_param.kMinimumSpacing = 1.0;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      default:
        // release 构建关闭 assert 后会保持 param 原值并继续返回成功。
        assert(false);
        break;
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif
