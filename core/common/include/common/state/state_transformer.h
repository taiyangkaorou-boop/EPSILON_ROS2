/**
 * @file state_transformer.h
 * @brief 声明世界坐标状态/点与参考车道 Frenet 表示之间的转换器。
 */
#ifndef _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__
#define _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__

#include "common/basics/basics.h"
#include "common/basics/config.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"

namespace common {
/**
 * @brief 在世界坐标 State/Point 与指定参考 Lane 的 Frenet 表示之间转换。
 *
 * 对象按值持有一条 Lane；所有转换都依赖该 Lane 有效，并使用 Lane 的弧长投影、
 * 切向、曲率及曲率导数。失败通过 ErrorType 返回，不抛出异常。
 */
class StateTransformer {
 public:
  /// 构造持有无效默认 Lane 的转换器。
  StateTransformer() {}
  /// 使用指定参考车道副本构造转换器。
  StateTransformer(const Lane& lane) { lane_ = lane; }

  /**
   * @brief 将 Frenet 状态转换为世界坐标车辆状态。
   * @param fs 输入 Frenet 状态，必须具有可用的 vec_ds。
   * @param s 输出世界坐标 State。
   * @return ErrorType Lane/状态非法或几何查询失败时返回错误。
   */
  ErrorType GetStateFromFrenetState(const FrenetState& fs, State* s) const;

  /**
   * @brief 将世界坐标车辆状态投影到参考车道 Frenet 坐标。
   * @param s 输入世界坐标 State。
   * @param fs 输出 FrenetState。
   * @return ErrorType 投影或车道几何查询失败时返回错误。
   * @note Lane 使用有限采样搜索弧长，位置可能引入约 1 cm 误差；历史测量耗时约
   * 0.03 ms，正式实验仍需重新统计。
   */
  ErrorType GetFrenetStateFromState(const State& s, FrenetState* fs) const;

  /// 批量把 State 转换为 FrenetState；任一元素失败即返回并保留此前结果。
  ErrorType GetFrenetStateVectorFromStates(const vec_E<State> state_vec,
                                           vec_E<FrenetState>* fs_vec) const;

  /// 批量把 FrenetState 转换为 State；任一元素失败即返回并保留此前结果。
  ErrorType GetStateVectorFromFrenetStates(const vec_E<FrenetState>& fs_vec,
                                           vec_E<State>* state_vec) const;

  /// 将单个世界坐标点转换为 `[s, d]` Frenet 点。
  ErrorType GetFrenetPointFromPoint(const Vec2f& s, Vec2f* fs) const;

  /// 批量转换世界坐标点；任一元素失败即返回并保留此前结果。
  ErrorType GetFrenetPointVectorFromPoints(const vec_E<Vec2f>& s,
                                           vec_E<Vec2f>* fs) const;

  /// 返回内部参考 Lane 是否已成功构造。
  bool IsValid() const { return lane_.IsValid(); }

  /// 预留调试接口；当前实现不输出任何内容。
  void print() {}

 private:
  /// 按值持有的参考车道几何。
  Lane lane_;
};

}  // namespace common

#endif
