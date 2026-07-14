#include "common/math/calculations.h"

// 对 0 到 5 使用常量快速返回，其余非负输入使用逐项累乘。
long long fac(int n) {
  if (n == 0) return 1;
  if (n == 1) return 1;
  if (n == 2) return 2;
  if (n == 3) return 6;
  if (n == 4) return 24;
  if (n == 5) return 120;

  long long ans = 1;
  for (int i = 1; i <= n; i++) ans *= i;
  return ans;
}

// 直接使用三个阶乘的整数商；调用方负责保证 0<=k<=n 且结果不溢出。
long long nchoosek(int n, int k) { return fac(n) / fac(k) / fac(n - k); }

// 只根据原始 theta 执行一次加减，适合已经接近标准角范围的状态更新结果。
decimal_t normalize_angle(const decimal_t& theta) {
  decimal_t theta_tmp = theta;
  theta_tmp -= (theta >= kPi) * 2 * kPi;
  theta_tmp += (theta < -kPi) * 2 * kPi;
  return theta_tmp;
}

// 应用标准二维旋转矩阵，angle 为正时逆时针旋转。
Vecf<2> rotate_vector_2d(const Vecf<2>& v, const decimal_t angle) {
  return Vecf<2>(v[0] * cos(angle) - v[1] * sin(angle),
                 v[0] * sin(angle) + v[1] * cos(angle));
}

// 零向量会按 atan2(0, 0) 的平台数学库行为返回结果，本层不单独拒绝。
decimal_t vec2d_to_angle(const Vecf<2>& v) { return atan2(v[1], v[0]); }

// 使用 max/min 完成闭区间截断；非法区间通过 assert 中止调试构建。
decimal_t truncate(const decimal_t& val_in, const decimal_t& lower,
                   const decimal_t& upper) {
  if (lower > upper) {
    printf("[Calculations]Invalid input!\n");
    assert(false);
  }
  decimal_t res = val_in;
  res = std::max(res, lower);
  res = std::min(res, upper);
  return res;
}

// 将截断后的相对位置按比例映射到目标区间；原区间零宽时会发生除零。
decimal_t normalize_with_bound(const decimal_t& val_in, const decimal_t& lower,
                               const decimal_t& upper,
                               const decimal_t& new_lower,
                               const decimal_t& new_upper) {
  if (new_lower > new_upper) {
    printf("[Calculations]Invalid input!\n");
    assert(false);
  }
  decimal_t val_bounded = truncate(val_in, lower, upper);
  decimal_t ratio = (val_bounded - lower) / (upper - lower);
  decimal_t res = new_lower + (new_upper - new_lower) * ratio;
  return res;
}

ErrorType RemapUsingQuadraticFuncAroundSmallValue(const decimal_t& th,
                                                  const decimal_t& val_in,
                                                  decimal_t* val_out) {
  // 令二次段在 |x|=|th| 附近与 y=x 的幅值衔接；负阈值不符合该设计假设。
  decimal_t c = 1.0 / th;
  if (fabs(val_in) <= fabs(th)) {
    // 零附近：y=sign(x)*x^2/th，降低小输入的输出幅值。
    *val_out = sgn(val_in) * c * val_in * val_in;
  } else {
    // 阈值外：保持输入不变。
    *val_out = val_in;
  }
  return kSuccess;
}
