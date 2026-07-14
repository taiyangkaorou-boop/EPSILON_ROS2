#ifndef _CORE_COMMON_INC_COMMON_SPLINE_POLYNOMIAL_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_POLYNOMIAL_H__

#include "common/basics/basics.h"
#include "common/math/calculations.h"
#include "common/spline/lookup_table.h"

#include <assert.h>

namespace common {

/**
 * @brief 使用“高阶在前、按阶乘缩放”系数约定的一维 N_DEG 次多项式。
 *
 * coeff_[N_DEG-i] 表示 s^i 项乘以 i! 后的系数，因此起点处的位置、速度、加速度等
 * 可直接落在末尾元素中；coeff_normal_order_ 则缓存普通幂基系数，供零阶快速求值。
 * 两种表示必须通过 update() 保持同步。
 */
template <int N_DEG>
class Polynomial {
 public:
  /// 固定长度的阶乘缩放系数向量。
  typedef Vecf<N_DEG + 1> VecNf;
  /// 根据 Eigen 固定尺寸对象大小决定是否启用对齐 new。
  enum { NeedsToAlign = (sizeof(VecNf) % 16) == 0 };

  /// 构造全零多项式，并同步清零两套系数表示。
  Polynomial() { set_zero(); }
  /// 使用阶乘缩放系数构造，并生成普通幂基缓存。
  Polynomial(const VecNf& coeff) : coeff_(coeff) { update(); }

  /// 返回阶乘缩放、最高阶在前的系数副本。
  VecNf coeff() const { return coeff_; }

  /// 写入阶乘缩放系数，并同步更新普通幂基缓存。
  void set_coeff(const VecNf& coeff) {
    coeff_ = coeff;
    update();
  }

  /// 根据 coeff_ 重新计算 coeff_normal_order_，下标 i 对应普通 s^i 系数。
  void update() {
    for (int i = 0; i < N_DEG + 1; i++) {
      coeff_normal_order_[i] = coeff_[N_DEG - i] / fac(i);
    }
  }

  /// 将阶乘缩放系数和普通幂基缓存同时置零。
  void set_zero() {
    coeff_.setZero();
    coeff_normal_order_.setZero();
  }

  /**
   * @brief 使用 Horner 法计算多项式对 s 的 d 阶导数。
   * @param s 求值参数。
   * @param d 导数阶数；baseline 要求 0<=d<=N_DEG。
   */
  inline decimal_t evaluate(const decimal_t& s, const int& d) const {
    // 阶乘缩放表示使求导只需改变每项除数，无需现场构造导数系数向量。
    decimal_t p = coeff_(0) / fac(N_DEG - d);
    for (int i = 1; i <= N_DEG - d; i++) {
      p = (p * s + coeff_(i) / fac(N_DEG - i - d));
    }
    return p;
    // 历史对照实现使用普通幂基缓存；测试显示当前写法通常更快。
    // decimal_t p = coeff_normal_order_[N_DEG] * fac(N_DEG) / fac(N_DEG - d);
    // for (int i = 1; i <= N_DEG - d; i++) {
    //   p = (p * s + coeff_normal_order_[N_DEG - i] * fac(N_DEG - i) /
    //                    fac(N_DEG - i - d));
    // }
    // return p;
  }

  /// 使用普通幂基缓存快速计算零阶多项式值，比 evaluate(s, 0) 路径更短。
  inline decimal_t evaluate(const decimal_t& s) const {
    // 普通幂基系数同样按最高阶到常数项执行 Horner 递推。
    decimal_t p = coeff_normal_order_[N_DEG];
    for (int i = 1; i <= N_DEG; i++) {
      p = (p * s + coeff_normal_order_[N_DEG - i]);
    }
    return p;
  }

  /**
   * @brief 返回从 0 到 s 的加速度平方或 jerk 平方解析积分原函数值。
   * @param s 积分上限。
   * @param d 只支持 2（加速度平方）或 3（jerk 平方）。
   * @note 公式按五次多项式的系数布局硬编码。
   */
  inline decimal_t J(decimal_t s, int d) const {
    if (d == 3) {
      // 五次多项式 jerk 的平方积分。
      return coeff_(0) * coeff_(0) / 20.0 * pow(s, 5) +
             coeff_(0) * coeff_(1) / 4 * pow(s, 4) +
             (coeff_(1) * coeff_(1) + coeff_(0) * coeff_(2)) / 3 * pow(s, 3) +
             coeff_(1) * coeff_(2) * s * s + coeff_(2) * coeff_(2) * s;
    } else if (d == 2) {
      // 五次多项式加速度的平方积分。
      return coeff_(0) * coeff_(0) / 252 * pow(s, 7) +
             coeff_(0) * coeff_(1) / 36 * pow(s, 6) +
             (coeff_(1) * coeff_(1) / 20 + coeff_(0) * coeff_(2) / 15) *
                 pow(s, 5) +
             (coeff_(0) * coeff_(3) / 12 + coeff_(1) * coeff_(2) / 4) *
                 pow(s, 4) +
             (coeff_(2) * coeff_(2) / 3 + coeff_(1) * coeff_(3) / 3) *
                 pow(s, 3) +
             coeff_(2) * coeff_(3) * s * s + coeff_(3) * coeff_(3) * s;
    } else {
      assert(false);
    }
    return 0.0;
  }

  /**
   * @brief 由两端位置、速度、加速度边界构造五次 jerk-optimal 连接。
   * @param p1 起点位置。
   * @param dp1 起点一阶导数。
   * @param ddp1 起点二阶导数。
   * @param p2 终点位置。
   * @param dp2 终点一阶导数。
   * @param ddp2 终点二阶导数。
   * @param S 参数时长，要求非负；多项式阶数必须至少为 5。
   */
  void GetJerkOptimalConnection(const decimal_t p1, const decimal_t dp1,
                                const decimal_t ddp1, const decimal_t p2,
                                const decimal_t dp2, const decimal_t ddp2,
                                const decimal_t S) {
    assert(N_DEG >= 5);
    Vecf<6> b;
    b << p1, dp1, ddp1, p2, dp2, ddp2;

    coeff_.setZero();

    // S 接近零时逆矩阵奇异，退化为仅编码起点二阶状态的低阶多项式。
    // baseline 在此直接返回，未调用 update() 刷新普通幂基缓存。
    if (S < kEPS) {
      Vecf<6> c;
      c << 0.0, 0.0, 0.0, ddp1, dp1, p1;
      coeff_.template segment<6>(N_DEG - 5) = c;
      return;
    }

    // 优先使用若干常见时长的预计算边界映射逆矩阵。
    MatNf<6> A_inverse;
    if (!LookUpCache(S, &A_inverse)) {
      A_inverse = GetAInverse(S);
    }

    // 求得普通五次幂基系数后，乘以相应阶乘写回项目内部表示。
    auto coeff = A_inverse * b;
    coeff_[N_DEG - 5] = coeff(0) * fac(5);
    coeff_[N_DEG - 4] = coeff(1) * fac(4);
    coeff_[N_DEG - 3] = coeff(2) * fac(3);
    coeff_[N_DEG - 2] = coeff(3) * fac(2);
    coeff_[N_DEG - 1] = coeff(4) * fac(1);
    coeff_[N_DEG - 0] = coeff(5);
    update();
  }

  /// 按时长精确匹配全局逆矩阵缓存，命中时复制到输出参数。
  bool LookUpCache(const decimal_t S, MatNf<6>* A_inverse) {
    auto it = kTableAInverse.find(S);
    if (it != kTableAInverse.end()) {
      *A_inverse = it->second;
      return true;
    } else {
      return false;
    }
  }

  /// 以固定 7 位小数打印阶乘缩放系数，供调试使用。
  void print() const {
    std::cout << std::fixed << std::setprecision(7) << coeff_.transpose()
              << std::endl;
  }

 private:
  /// 阶乘缩放且最高阶在前的主系数表示。
  VecNf coeff_;
  /// 普通幂基且常数项在前的零阶求值缓存。
  VecNf coeff_normal_order_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

/**
 * @brief 将 N_DIM 个独立一维多项式组合为固定维向量多项式。
 *
 * 各维共享相同参数 s 和导数阶数，但系数相互独立；本类不表达维度间耦合约束。
 */
template <int N_DEG, int N_DIM>
class PolynomialND {
 public:
  /// 默认构造 N_DIM 个全零一维多项式。
  PolynomialND() {}
  /// 从各维一维多项式数组按值构造。
  PolynomialND(const std::array<Polynomial<N_DEG>, N_DIM>& polys)
      : polys_(polys) {}

  /**
   * @brief 返回指定维的一维多项式可写引用。
   * @param j 维度下标；baseline 只检查上界。
   */
  Polynomial<N_DEG>& operator[](int j) {
    assert(j < N_DIM);
    return polys_[j];
  }

  /**
   * @brief 逐维计算 d 阶导数并写入输出向量。
   * @param s 求值参数。
   * @param d 导数阶数。
   * @param vec 输出固定维向量。
   */
  inline void evaluate(const decimal_t s, int d, Vecf<N_DIM>* vec) const {
    for (int i = 0; i < N_DIM; i++) {
      (*vec)[i] = polys_[i].evaluate(s, d);
    }
  }

  /// 逐维调用一维多项式的零阶快速求值接口。
  inline void evaluate(const decimal_t s, Vecf<N_DIM>* vec) const {
    for (int i = 0; i < N_DIM; i++) {
      (*vec)[i] = polys_[i].evaluate(s);
    }
  }

  /// 按维度顺序打印所有一维多项式的阶乘缩放系数。
  void print() const {
    for (int i = 0; i < N_DIM; i++) polys_[i].print();
  }

 private:
  /// 各输出维度对应的一维多项式数组。
  std::array<Polynomial<N_DEG>, N_DIM> polys_;
};

}  // namespace common

#endif
