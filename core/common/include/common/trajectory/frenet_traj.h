#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__

#include "common/basics/config.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/trajectory/trajectory.h"

namespace common {

/**
 * @brief 同时支持 Frenet 状态和世界状态查询的轨迹抽象接口。
 *
 * 世界状态通常通过派生类持有的 StateTransformer 从 Frenet 表示转换得到。
 */
class FrenetTrajectory : public Trajectory {
 public:
  virtual ~FrenetTrajectory() = default;
  /// 查询世界坐标车辆状态。
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;
  /// 查询纵向/横向 Frenet 状态。
  virtual ErrorType GetFrenetState(const decimal_t& t,
                                   FrenetState* fs) const = 0;
  /// 返回轨迹参数域起点。
  virtual decimal_t begin() const = 0;
  /// 返回轨迹参数域终点。
  virtual decimal_t end() const = 0;
  /// 返回派生轨迹是否有效。
  virtual bool IsValid() const = 0;
  /// 返回派生类定义的优化变量副本。
  virtual std::vector<decimal_t> variables() const = 0;
  /// 按派生类编码覆盖优化变量。
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;
  /// 输出派生类定义的纵向与横向 jerk 指标；当前 primitive 实现使用平方积分。
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const = 0;
};

}  // namespace common

#endif
