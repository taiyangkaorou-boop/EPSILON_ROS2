/**
 * @file tool_func.h
 * @author HKUST Aerial Robotics Group
 * @brief 声明字符串切分、笛卡尔积、格式化和等间隔采样等通用工具。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__
#define _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "common/basics/basics.h"

namespace common {

/**
 * @brief 按指定分隔符切分字符串，并把非尾部空结果追加到输出容器。
 *
 * @param s 输入字符串。
 * @param c 非空分隔符。
 * @param v 输出片段容器；函数不会主动清空已有内容。
 */
void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v);

/**
 * @brief 递归枚举整数向量集合的笛卡尔积。
 *
 * @param vec 每一层可选整数集合。
 * @param N 当前递归层下标，首次调用应为 0。
 * @param tmp 当前组合的回溯缓存。
 * @param tmp_result 完整组合输出容器。
 */
void GetResultInVector(const std::vector<std::vector<int>>& vec, const int& N,
                       std::vector<int>* tmp,
                       std::vector<std::vector<int>>* tmp_result);
/**
 * @brief 生成输入整数集合之间的全部组合。
 *
 * @param vec_in 非空候选集合数组。
 * @param res 输出组合容器；函数不会主动清空已有内容。
 */
void GetAllCombinations(const std::vector<std::vector<int>>& vec_in,
                        std::vector<std::vector<int>>* res);

/**
 * @brief 使用 fixed 格式和指定小数位数将数值转换为字符串。
 *
 * @tparam T 支持流输出运算符的数值类型。
 * @param val 待格式化数值。
 * @param pre 小数位数。
 * @return std::string 格式化字符串。
 */
template <typename T>
std::string GetStringByValueWithPrecision(const T& val, const int& pre) {
  std::ostringstream os;
  os << std::fixed;
  os << std::setprecision(pre);
  os << val;
  return os.str();
}

/**
 * @brief 生成从下界开始、按固定步长递增的采样序列。
 * @tparam T 支持加减乘除和 ceil 计算的数值类型。
 * @param lb 下界，始终作为首个采样候选。
 * @param ub 上界。
 * @param step 正步长；调用方必须保证大于零。
 * @param if_inc_tail 是否无条件在末尾加入 ub。
 * @param vec 输出容器，函数会先清空。
 */
template <typename T>
void GetRangeVector(const T& lb, const T& ub, const T& step,
                    const bool& if_inc_tail, std::vector<T>* vec) {
  vec->clear();
  // 减去 kEPS 避免浮点舍入使本应位于上界的点被重复计数。
  int num = std::ceil((ub - lb - kEPS) / step);
  for (int i = 0; i < num; ++i) {
    vec->push_back(lb + i * step);
  }
  if (if_inc_tail) {
    vec->push_back(ub);
  }
}

}  // namespace common

#endif  // _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__
