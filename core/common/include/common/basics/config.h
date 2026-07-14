/**
 * @file config.h
 * @author HKUST Aerial Robotics Group
 * @brief 定义车道和轨迹多项式的统一阶数与空间维度。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_CONFIG_H__
#define _CORE_COMMON_INC_BASICS_CONFIG_H__

// 车道中心线使用五次二维多项式表示。
#define LaneDegree 5
#define LaneDim 2
// 连续轨迹同样使用五次二维多项式，便于复用样条求解工具。
#define TrajectoryDegree 5
#define TrajectoryDim 2

#endif  // _CORE_COMMON_INC_BASICS_CONFIG_H__
