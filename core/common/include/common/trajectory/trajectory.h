#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__

#include "common/basics/config.h"
#include "common/state/state.h"

namespace common {

/**
 * @brief 可按全局参数查询世界坐标车辆状态的轨迹抽象接口。
 *
 * 派生类负责定义参数域、有效性和内部优化变量编码；接口不规定采样方式、外推行为或
 * 线程安全性。
 */
class Trajectory {
 public:
  /// 通过虚析构保证经基类指针销毁派生轨迹安全。
  virtual ~Trajectory() = default;
  /// 查询全局参数 t 处的世界坐标车辆状态。
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;
  /// 返回轨迹参数域起点。
  virtual decimal_t begin() const = 0;
  /// 返回轨迹参数域终点。
  virtual decimal_t end() const = 0;
  /// 返回派生对象是否完成有效初始化。
  virtual bool IsValid() const = 0;
  /// 返回供外部优化器读写的内部变量副本。
  virtual std::vector<decimal_t> variables() const = 0;
  /// 使用派生类约定的编码覆盖内部优化变量。
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;
};

}  // namespace common

#endif
