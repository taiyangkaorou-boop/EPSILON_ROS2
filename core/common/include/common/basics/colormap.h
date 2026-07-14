/**
 * @file colormap.h
 * @author HKUST Aerial Robotics Group
 * @brief 声明可视化使用的 ARGB 颜色类型、离散色表和数值到颜色的映射函数。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_COLORMAP_H__
#define _COMMON_INC_COMMON_BASICS_COLORMAP_H__

#include <map>

#include "common/basics/basics.h"

namespace common {
/// 使用 [0, 1] 浮点通道表示的透明度、红、绿、蓝颜色。
struct ColorARGB {
  decimal_t a;
  decimal_t r;
  decimal_t g;
  decimal_t b;
  /// 按透明度、红、绿、蓝顺序构造颜色。
  ColorARGB(decimal_t A, decimal_t R, decimal_t G, decimal_t B)
      : a(A), r(R), g(G), b(B) {}
  /// 默认构造全透明黑色。
  ColorARGB() : a(0.0), r(0.0), g(0.0), b(0.0) {}

  /// 返回仅替换透明度的新颜色，不修改当前对象。
  ColorARGB set_a(const decimal_t _a) { return ColorARGB(_a, r, g, b); }
};

// 具名颜色表及两个以归一化数值为键的查找表，在 colormap.cc 中定义。
extern std::map<std::string, ColorARGB> cmap;
extern std::map<decimal_t, ColorARGB> jet_map;
extern std::map<decimal_t, ColorARGB> autumn_map;

/**
 * @brief 在有序查找表中取得第一个键严格大于输入值的颜色。
 *
 * @param val 输入标量；调用方应保证其小于色表最大键。
 * @param m 非空的有序颜色查找表。
 * @return ColorARGB 对应上界区间的颜色。
 */
ColorARGB GetColorByValue(const decimal_t val,
                          const std::map<decimal_t, ColorARGB>& m);

/**
 * @brief 将输入值截断到给定范围并映射为分段线性的 Jet 颜色。
 *
 * @param val 输入标量。
 * @param max 映射范围上界。
 * @param min 映射范围下界；必须严格小于 max。
 * @return ColorARGB 不透明的 Jet 颜色。
 */
ColorARGB GetJetColorByValue(const decimal_t val, const decimal_t max,
                             const decimal_t min);
}  // namespace common

#endif
