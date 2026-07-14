/**
 * @file free_state.h
 * @brief 声明世界坐标二维矢量状态及其与车辆标量 State 的转换接口。
 */
#ifndef _COMMON_INC_COMMON_STATE_FREE_STATE_H__
#define _COMMON_INC_COMMON_STATE_FREE_STATE_H__

#include "common/basics/basics.h"
#include "common/state/state.h"

#include <math.h>
namespace common {
/**
 * @brief 世界坐标系中的二维矢量运动状态。
 *
 * 与 State 的标量速度/切向加速度表示不同，FreeState 直接保存二维速度和加速度，
 * 便于执行坐标变换或自由空间动力学计算。
 */
struct FreeState {
  decimal_t time_stamp{0.0};
  Vecf<2> position{Vecf<2>::Zero()};
  Vecf<2> velocity{Vecf<2>::Zero()};
  Vecf<2> acceleration{Vecf<2>::Zero()};
  decimal_t angle{0.0};

  /// 输出位置、速度、加速度和航向，供调试使用。
  void print() const {
    printf("position: (%lf, %lf).\n", position[0], position[1]);
    printf("velocity: (%lf, %lf).\n", velocity[0], velocity[1]);
    printf("acceleration: (%lf, %lf).\n", acceleration[0], acceleration[1]);
    printf("angle: %lf.\n", angle);
  }
};

/**
 * @brief 将 State 的标量运动量展开为世界坐标二维向量。
 * @param state 输入后轴中心状态。
 * @param free_state 输出自由空间状态。
 */
void GetFreeStateFromState(const State& state, FreeState* free_state);
/**
 * @brief 将世界坐标二维速度/加速度投影回 State 的切向标量表示。
 * @param free_state 输入自由空间状态。
 * @param state 输出状态；速度使用向量模长，因此不能恢复倒车速度符号。
 */
void GetStateFromFreeState(const FreeState& free_state, State* state);
}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_FREE_STATE_H__
