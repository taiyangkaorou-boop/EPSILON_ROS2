/**
 * @file planner.h
 * @brief 路径、行为和轨迹规划器共享的最小生命周期抽象接口。
 */

#ifndef _COMMON_INC_COMMON_INTERFACE_PLANNER_H__
#define _COMMON_INC_COMMON_INTERFACE_PLANNER_H__

#include <string>

#include "common/basics/basics.h"

namespace planning {

/**
 * @brief 路径、运动和行为规划器的通用基类。
 *
 * 派生类提供名称、字符串配置初始化和单周期运行；具体输入/输出通过派生类 setter/getter 注入，
 * ErrorType 只表达成功或失败类别。虚析构保证通过基类指针销毁派生对象安全。
 */
class Planner {
 public:
  /// 默认构造不隐式初始化派生类资源。
  Planner() = default;

  /// 支持通过 Planner 指针安全销毁派生规划器。
  virtual ~Planner() = default;

  /// 返回规划器显示/日志名称。
  virtual std::string Name() = 0;

  /// 使用派生类约定的配置字符串初始化。
  virtual ErrorType Init(const std::string config) = 0;

  /// 在已注入的最新输入上执行一个规划周期。
  virtual ErrorType RunOnce() = 0;
};

}  // namespace planning

#endif
