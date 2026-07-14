/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief 定义全项目共享的数值类型、Eigen 别名、误差码和基础常量。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_BASICS_H_
#define _CORE_COMMON_INC_BASICS_BASICS_H_

#include "common/basics/macros.h"
#include "common/basics/tic_toc.h"

#include <math.h>
#include <stdio.h>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#define BACKWARD_HAS_UNWIND 1
#define BACKWARD_HAS_DW 1
#include "backward.hpp"

/// 跨模块函数的轻量返回状态；具体错误上下文仍应由调用者记录日志。
enum ErrorType { kSuccess = 0, kWrongStatus, kIllegalInput, kUnknown };

/// 规划系统统一使用双精度浮点数，避免不同模块间发生隐式精度转换。
using decimal_t = double;

// 固定/动态维度 Eigen 向量与矩阵别名，统一项目内线性代数接口。
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;

template <int N>
using Veci = Eigen::Matrix<int, N, 1>;

template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;

template <int M, int N>
using Mati8 = Eigen::Matrix<uint8_t, M, N>;

template <int M, int N>
using Mati = Eigen::Matrix<int, M, N>;

template <int N>
using MatNf = Matf<N, N>;

template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;

using MatDf = Matf<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi = Mati<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi8 = Mati8<Eigen::Dynamic, Eigen::Dynamic>;

using Mat2f = Matf<2, 2>;
using Mat3f = Matf<3, 3>;
using Mat4f = Matf<4, 4>;

using Vec2f = Vecf<2>;
using Vec3f = Vecf<3>;
using Vec4f = Vecf<4>;

using Vec2i = Veci<2>;
using Vec3i = Veci<3>;
using Vec4i = Veci<4>;

// 固定尺寸 Eigen 类型需要对齐分配器，才能安全存入 STL 容器。
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N>
using vec_Vecf = vec_E<Vecf<N>>;

// 不同数量级的容差分别用于宽松几何判断、常规数值比较和近零判断。
const decimal_t kBigEPS = 1e-1;

const decimal_t kEPS = 1e-6;

const decimal_t kSmallEPS = 1e-10;

/// 圆周率常量，由标准反余弦函数计算以避免硬编码截断。
const decimal_t kPi = acos(-1.0);

/// 代替数学无穷大的有限哨兵值，供代价和边界初始化使用。
const decimal_t kInf = 1e20;

// 负值 ID 表示尚未绑定有效车辆或车道。
const int kInvalidAgentId = -1;
const int kInvalidLaneId = -1;

#endif  // CORE_COMMON_INC_BASICS_BASICS_H
