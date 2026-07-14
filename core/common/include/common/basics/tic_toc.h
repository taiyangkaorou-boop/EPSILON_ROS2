/**
 * @file tic_toc.h
 * @author HKUST Aerial Robotics Group
 * @brief 提供毫秒级耗时统计和 system_clock 时间点转换工具。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_TIC_TOC_H_
#define _COMMON_INC_COMMON_BASICS_TIC_TOC_H_

#include <chrono>
#include <cstdlib>
#include <ctime>

class TicToc {
 public:
  /// 构造时立即开始一次计时。
  TicToc() { tic(); }

  /**
   * @brief 重置起始时间点。
   * @note toc() 的返回单位为毫秒。
   */
  void tic() { start = std::chrono::system_clock::now(); }

  /**
   * @brief 记录结束时间并返回自最近一次 tic() 以来的耗时。
   * @return double 毫秒数。
   */
  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

  /**
   * @brief 将 system_clock 时间点转换为 Unix epoch 起算的秒数。
   *
   * @param t 待转换时间点。
   * @return double 自 epoch 起的秒数。
   */
  static double TimePointToDouble(
      const std::chrono::system_clock::time_point& t) {
    auto tt = std::chrono::duration<double>(t.time_since_epoch());
    return tt.count();
  }

 private:
  // start 为最近一次 tic() 的时间，end 为最近一次 toc() 的采样时间。
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif
