/**
 * @file macros.h
 * @author HKUST Aerial Robotics Group
 * @brief 定义跨模块复用的编译期宏，目前仅封装 backward-cpp 信号处理器。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_MACROS_H__
#define _COMMON_INC_COMMON_MACROS_H__
// 在可执行文件的一个翻译单元中展开该宏，注册崩溃信号和堆栈回溯处理。
#define DECLARE_BACKWARD       \
  namespace backward {         \
  backward::SignalHandling sh; \
  }
#endif
