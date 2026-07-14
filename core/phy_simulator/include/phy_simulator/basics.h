/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief 物理仿真模块的公共类型依赖入口。
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_BASICS_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_BASICS_H_

#include <assert.h>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

// 当前文件只集中引入仿真器常用的 common/Eigen 类型，暂未定义模块自有基础类型。
namespace phy_simulator {}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_BASICS_H_
