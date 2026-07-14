#ifndef _COMMON_INC_COMMON_SPLINE_LOOKUP_TABLE_H__
#define _COMMON_INC_COMMON_SPLINE_LOOKUP_TABLE_H__

#include <map>
#include "common/basics/basics.h"
#include "common/math/calculations.h"

namespace common {

/**
 * @brief 计算五次普通幂基系数到两端位置/速度/加速度边界的映射逆矩阵。
 * @param t 终端参数时长；t=0 时矩阵奇异。
 * @return 6x6 边界到普通幂基系数的逆映射。
 */
MatNf<6> GetAInverse(decimal_t t);

/// 常见时长到逆矩阵的全局预计算表；键使用浮点精确比较，可被外部代码修改。
extern std::map<decimal_t, MatNf<6>, std::less<decimal_t>,
                Eigen::aligned_allocator<std::pair<const decimal_t, MatNf<6>>>>
    kTableAInverse;

}  // namespace common

#endif
