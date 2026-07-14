/**
 * @file colormap.cc
 * @author HKUST Aerial Robotics Group
 * @brief 定义具名色表、Jet/Autumn 查找表及连续 Jet 映射实现。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/colormap.h"

namespace common {
// 常用具名颜色，主要供 ROS/RViz 可视化模块按语义选择颜色。
std::map<std::string, ColorARGB> cmap{
    {"black", ColorARGB(1.0, 0.0, 0.0, 0.0)},
    {"white", ColorARGB(1.0, 1.0, 1.0, 1.0)},
    {"red", ColorARGB(1.0, 1.0, 0.0, 0.0)},
    {"hot pink", ColorARGB(1.0, 1.0, 0.44, 0.70)},
    {"green", ColorARGB(1.0, 0.0, 1.0, 0.0)},
    {"blue", ColorARGB(1.0, 0.0, 0.0, 1.0)},
    {"aqua marine", ColorARGB(1.0, 0.5, 1.0, 0.83)},
    {"yellow", ColorARGB(1.0, 1.0, 1.0, 0.0)},
    {"cyan", ColorARGB(1.0, 0.0, 1.0, 1.0)},
    {"magenta", ColorARGB(1.0, 1.0, 0.0, 1.0)},
    {"violet", ColorARGB(1.0, 0.93, 0.43, 0.93)},
    {"orange red", ColorARGB(1.0, 1.0, 0.275, 0.0)},
    {"orange", ColorARGB(1.0, 1.0, 0.65, 0.0)},
    {"dark orange", ColorARGB(1.0, 1.0, 0.6, 0.0)},
    {"gold", ColorARGB(1.0, 1.0, 0.84, 0.0)},
    {"green yellow", ColorARGB(1.0, 0.5, 1.0, 0.0)},
    {"forest green", ColorARGB(1.0, 0.13, 0.545, 0.13)},
    {"spring green", ColorARGB(1.0, 0.0, 1.0, 0.5)},
    {"sky blue", ColorARGB(1.0, 0.0, 0.749, 1.0)},
    {"medium orchid", ColorARGB(1.0, 0.729, 0.333, 0.827)},
    {"grey", ColorARGB(1.0, 0.5, 0.5, 0.5)}};

// Jet 色表按 0.1 间隔离散采样，键表示归一化强度。
std::map<decimal_t, ColorARGB> jet_map{
    {0.0, ColorARGB(1.0, 0.0, 0.0, 0.6667)},
    {0.1, ColorARGB(1.0, 0.0, 0.0, 1.0000)},
    {0.2, ColorARGB(1.0, 0.0, 0.3333, 1.0000)},
    {0.3, ColorARGB(1.0, 0.0, 0.6667, 1.0000)},
    {0.4, ColorARGB(1.0, 0.0, 1.0000, 1.0000)},
    {0.5, ColorARGB(1.0, 0.3333, 1.0000, 0.6667)},
    {0.6, ColorARGB(1.0, 0.6667, 1.0000, 0.3333)},
    {0.7, ColorARGB(1.0, 1.0000, 1.0000, 0.0)},
    {0.8, ColorARGB(1.0, 1.0000, 0.6667, 0.0)},
    {0.9, ColorARGB(1.0, 1.0000, 0.3333, 0.0)},
    {1.0, ColorARGB(1.0, 1.0000, 0.0, 0.0)}};

// Autumn 色表从红色平滑过渡到黄色，透明度固定为 0.5。
std::map<decimal_t, ColorARGB> autumn_map{
    {1.0, ColorARGB(0.5, 1.0, 0.0, 0.0)},
    {0.95, ColorARGB(0.5, 1.0, 0.05, 0.0)},
    {0.9, ColorARGB(0.5, 1.0, 0.1, 0.0)},
    {0.85, ColorARGB(0.5, 1.0, 0.15, 0.0)},
    {0.8, ColorARGB(0.5, 1.0, 0.2, 0.0)},
    {0.75, ColorARGB(0.5, 1.0, 0.25, 0.0)},
    {0.7, ColorARGB(0.5, 1.0, 0.3, 0.0)},
    {0.65, ColorARGB(0.5, 1.0, 0.35, 0.0)},
    {0.6, ColorARGB(0.5, 1.0, 0.4, 0.0)},
    {0.55, ColorARGB(0.5, 1.0, 0.45, 0.0)},
    {0.5, ColorARGB(0.5, 1.0, 0.5, 0.0)},
    {0.45, ColorARGB(0.5, 1.0, 0.55, 0.0)},
    {0.4, ColorARGB(0.5, 1.0, 0.6, 0.0)},
    {0.35, ColorARGB(0.5, 1.0, 0.65, 0.0)},
    {0.3, ColorARGB(0.5, 1.0, 0.7, 0.0)},
    {0.25, ColorARGB(0.5, 1.0, 0.75, 0.0)},
    {0.2, ColorARGB(0.5, 1.0, 0.8, 0.0)},
    {0.15, ColorARGB(0.5, 1.0, 0.85, 0.0)},
    {0.1, ColorARGB(0.5, 1.0, 0.9, 0.0)},
    {0.05, ColorARGB(0.5, 1.0, 0.95, 0.0)},
    {0.0, ColorARGB(0.5, 1.0, 1.0, 0.0)}};

ColorARGB GetColorByValue(const decimal_t val,
                          const std::map<decimal_t, ColorARGB>& m) {
  // upper_bound 实现右开区间查找；输入越过最大键时不能解引用 end()。
  auto it = m.upper_bound(val);
  return it->second;
}

ColorARGB GetJetColorByValue(const decimal_t val_in, const decimal_t vmax,
                             const decimal_t vmin) {
  // 先截断输入，确保后续四段线性映射始终落在指定数值范围内。
  decimal_t val = val_in;
  if (val < vmin) val = vmin;
  if (val > vmax) val = vmax;
  double dv = vmax - vmin;

  // 四个四分位区间依次完成蓝→青→黄→红的通道插值。
  ColorARGB c(1.0, 1.0, 1.0, 1.0);
  if (val < (vmin + 0.25 * dv)) {
    c.r = 0;
    c.g = 4.0 * (val - vmin) / dv;
  } else if (val < (vmin + 0.5 * dv)) {
    c.r = 0;
    c.b = 1.0 + 4.0 * (vmin + 0.25 * dv - val) / dv;
  } else if (val < (vmin + 0.75 * dv)) {
    c.r = 4.0 * (val - vmin - 0.5 * dv) / dv;
    c.b = 0.0;
  } else {
    c.g = 1 + 4.0 * (vmin + 0.75 * dv - val) / dv;
    c.b = 0.0;
  }
  return c;
}

}  // namespace common
