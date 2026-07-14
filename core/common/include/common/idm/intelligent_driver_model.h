#ifndef _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__
#define _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__

#include "common/basics/basics.h"

namespace common {

/**
 * @brief IDM、改进 IDM（IIDM）和自适应巡航 ACC 纵向加速度公式的无状态工具。
 *
 * 模型输入是一维纵向点状态，输出期望加速度；不负责时间积分、车辆动力学约束或参数
 * 校验。vehicle_model 和 MOBIL 均复用这里的静态公式。
 */
class IntelligentDriverModel {
 public:
  /// 某一时刻自车和前车的纵向位置/速度输入。
  struct State {
    /// 自车纵向位置。
    decimal_t s{0.0};
    /// 自车纵向速度。
    decimal_t v{0.0};
    /// 前车纵向位置。
    decimal_t s_front{0.0};
    /// 前车纵向速度。
    decimal_t v_front{0.0};

    /// 构造全零状态。
    State() {}
    /// 由自车/前车位置与速度构造状态。
    State(const decimal_t &s_, const decimal_t &v_, const decimal_t &s_front_,
          const decimal_t &v_front_)
        : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
  };

  /**
   * @brief 三种纵向模型共享的驾驶风格与车辆参数。
   *
   * 加速和制动字段均按正的幅值解释。默认期望速度为 0，直接用于模型会产生除零风险。
   */
  struct Param {
    /// 自由流期望速度 v0。
    decimal_t kDesiredVelocity = 0.0;
    /// 用于把前后位置差换算为净间距的固定前车长度。
    decimal_t kVehicleLength = 5.0;
    /// 静止最小间距 s0。
    decimal_t kMinimumSpacing = 2.0;
    /// 期望时距 T。
    decimal_t kDesiredHeadwayTime = 1.0;
    /// 最大自由加速度 a。
    decimal_t kAcceleration = 2.0;
    /// 舒适制动减速度幅值 b。
    decimal_t kComfortableBrakingDeceleration = 3.0;
    /// 输出加速度的最大硬制动幅值。
    decimal_t kHardBrakingDeceleration = 5.0;
    /// 自由流速度项指数 delta。
    int kExponent = 4;
  };

  /// 计算原始 IDM 期望加速度，不对输出做硬制动截断。
  static ErrorType GetIdmDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);
  /// 计算改善期望速度附近稳态间距和超速制动行为的 IIDM 加速度，并做幅值截断。
  static ErrorType GetIIdmDesiredAcceleration(const Param &param,
                                              const State &cur_state,
                                              decimal_t *acc);
  /// 以 IIDM 为基础，融合 constant-acceleration heuristic 得到 ACC 加速度。
  static ErrorType GetAccDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);
};

}  // namespace common

#endif
