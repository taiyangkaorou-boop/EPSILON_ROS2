#ifndef _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__
#define _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__

#include "common/basics/basics.h"

/**
 * @brief 返回数值符号。
 * @tparam T 支持与零比较的数值类型。
 * @param val 输入值。
 * @return 正数返回 1，负数返回 -1，零返回 0。
 */
template <typename T>
int sgn(const T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief 计算整数阶乘 n!。
 * @param n 输入整数；baseline 预期 n 为非负小整数。
 * @return 使用 long long 保存的阶乘结果。
 */
long long fac(int n);

/**
 * @brief 通过 n!/(k!(n-k)!) 计算组合数。
 * @param n 总元素数。
 * @param k 选择元素数。
 */
long long nchoosek(int n, int k);

/**
 * @brief 对角度执行一次 2*pi 加减，使常规输入落入 [-pi, pi)。
 * @param theta 输入弧度。
 * @return 调整后的弧度；实现不是任意大角度的循环取模。
 */
decimal_t normalize_angle(const decimal_t& theta);

/**
 * @brief 将二维向量逆时针旋转指定弧度。
 * @param v 输入向量。
 * @param angle 旋转角，单位为弧度。
 * @return 旋转后的二维向量。
 */
Vecf<2> rotate_vector_2d(const Vecf<2>& v, const decimal_t angle);

/**
 * @brief 使用 atan2 返回二维向量的方向角。
 * @param v 输入向量。
 * @return 位于 atan2 约定范围内的弧度。
 */
decimal_t vec2d_to_angle(const Vecf<2>& v);

/**
 * @brief 将输入值截断到闭区间 [lower, upper]。
 * @param val_in 输入值。
 * @param lower 下界。
 * @param upper 上界。
 */
decimal_t truncate(const decimal_t& val_in, const decimal_t& lower,
                   const decimal_t& upper);

/**
 * @brief 先截断输入，再把原区间线性映射到新闭区间。
 * @param val_in 输入值。
 * @param lower 原区间下界。
 * @param upper 原区间上界。
 * @param new_lower 目标区间下界。
 * @param new_upper 目标区间上界。
 */
decimal_t normalize_with_bound(const decimal_t& val_in, const decimal_t& lower,
                               const decimal_t& upper,
                               const decimal_t& new_lower,
                               const decimal_t& new_upper);

/**
 * @brief 在零附近使用带符号二次函数压缩小量，阈值外保持恒等映射。
 * @param th 二次段阈值；baseline 预期为非零正数。
 * @param val_in 输入值。
 * @param val_out 输出映射值。
 * @return 当前实现固定返回 kSuccess。
 */
ErrorType RemapUsingQuadraticFuncAroundSmallValue(const decimal_t& th,
                                                  const decimal_t& val_in,
                                                  decimal_t* val_out);

#endif  // _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__
