#ifndef _CORE_SEMANTIC_MAP_MANAGER_INC_TRAFFIC_SIGNAL_MANAGER_H__
#define _CORE_SEMANTIC_MAP_MANAGER_INC_TRAFFIC_SIGNAL_MANAGER_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"

namespace semantic_map_manager {

/**
 * @brief 管理静态限速/交通灯列表，并判断信号对给定 Lane 上车辆状态是否生效。
 *
 * 信号端点先投影到参考 Lane 的 Frenet 坐标；限速在车辆已进入控制区，或以固定减速度
 * 估算的制动距离已经覆盖到信号起点时生效。当前交通灯停车状态仍为预留接口。
 */
class TrafficSignalManager {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using SpeedLimit = common::SpeedLimit;
  using TrafficLight = common::TrafficLight;

  /// 车辆相对信号纵向区间的位置关系。
  enum IntersectionType { kNotIntersect = 0, kSignalAhead, kSignalControlled };

  /// 构造时立即调用 LoadSignals；返回码不向外暴露。
  TrafficSignalManager();

  /// 预留初始化入口；当前为空操作并固定返回成功。
  ErrorType Init();

  /// 加载场景信号；当前所有硬编码示例均被注释，默认不会加入任何信号。
  ErrorType LoadSignals();

  /// 永久删除当前时刻不在有效时间区间内的限速信号。
  ErrorType UpdateSignals(const decimal_t time_elapsed);

  /// 返回给定状态/Lane 当前应服从的最小最大限速；无生效信号时输出 kInf。
  ErrorType GetSpeedLimit(const State& state, const Lane& lane,
                          decimal_t* speed_limit) const;

  /// 查询交通信号对应停车状态；当前为空实现，不写 stopping_state 但返回成功。
  ErrorType GetTrafficStoppingState(const State& state, const Lane& lane,
                                    State* stopping_state) const;

  /// 返回当前限速信号列表的值拷贝。
  inline vec_E<SpeedLimit> speed_limit_list() const {
    return speed_limit_list_;
  }
  /// 返回当前交通灯列表的值拷贝。
  inline vec_E<TrafficLight> traffic_light_list() const {
    return traffic_light_list_;
  }

 private:
  /// 投影信号首尾端点并判断车辆位于信号前、控制区内或不相交，同时输出纵向距离。
  ErrorType CheckIntersectionTypeWithSignal(const common::FrenetState& fs,
                                            const Lane& lane,
                                            const common::TrafficSignal& signal,
                                            IntersectionType* intersection_type,
                                             decimal_t* dist_to_startpt,
                                             decimal_t* dist_to_endpt) const;

  // 当前存活的限速与交通灯信号；UpdateSignals 只处理限速列表。
  vec_E<SpeedLimit> speed_limit_list_;
  vec_E<TrafficLight> traffic_light_list_;
};

}  // namespace semantic_map_manager
#endif
