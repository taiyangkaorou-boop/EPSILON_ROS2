#ifndef _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__

#include "common/spline/polynomial.h"

#include <assert.h>
#include <vector>

namespace common {

/**
 * @brief 由多个局部向量多项式组成的固定阶数、固定维数分段样条。
 *
 * vec_domain_ 保存全局参数断点，每段 PolynomialND 使用局部参数
 * h=s-vec_domain_[segment]。本类只负责分段选择和求值，不自动施加段间连续性；连续
 * 约束由 SplineGenerator 构造系数时保证。
 */
template <int N_DEG, int N_DIM>
class Spline {
 public:
  /// 单个分段的 N_DIM 维多项式类型。
  typedef PolynomialND<N_DEG, N_DIM> PolynomialType;

  /// 构造空参数域、空分段样条。
  Spline() {}

  /**
   * @brief 设置全局参数断点，并按“断点数减一”重建空白分段数组。
   * @param vec_domain 输入断点；baseline 只断言数量大于 1，不检查严格递增。
   */
  void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
    assert(vec_domain.size() > 1);
    vec_domain_ = vec_domain;
    poly_.resize(vec_domain_.size() - 1);
  }

  /// 返回当前多项式分段数量。
  int num_segments() const { return static_cast<int>(poly_.size()); }

  /// 返回全局参数断点副本。
  std::vector<decimal_t> vec_domain() const { return vec_domain_; }

  /// 返回参数域起点；空样条返回 0。
  decimal_t begin() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.front();
  }

  /// 返回参数域终点；空样条返回 0。
  decimal_t end() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.back();
  }

  /**
   * @brief 返回指定分段、指定维度的一维多项式可写引用。
   * @param n 分段下标。
   * @param j 维度下标。
   * @note baseline 只断言两个下标小于上界，未检查负数。
   */
  Polynomial<N_DEG>& operator()(int n, int j) {
    assert(n < this->num_segments());
    assert(j < N_DIM);
    return poly_[n][j];
  }

  /**
   * @brief 查询样条在全局参数 s 处的 d 阶导数向量。
   * @param s 全局求值参数。
   * @param d 导数阶数。
   * @param ret 输出向量。
   * @note 参数越界时自动使用首段或末段外推，并仍返回 kSuccess。
   */
  ErrorType evaluate(const decimal_t s, int d, Vecf<N_DIM>* ret) const {
    // 只有完全空域会被拒绝；单断点但无分段的异常对象未在此保护。
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    // 内部断点精确命中时选择左侧分段，在其局部终点处求值。
    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
    decimal_t h = s - vec_domain_[idx];

    if (s < vec_domain_[0]) {
      // 左外推使用第一段及相对首断点的负局部参数。
      poly_[0].evaluate(h, d, ret);
    } else if (s > vec_domain_[num_pts - 1]) {
      // 右外推使用最后一段及相对最后一段起点的局部参数。
      h = s - vec_domain_[idx - 1];
      poly_[num_pts - 2].evaluate(h, d, ret);
    } else {
      poly_[idx].evaluate(h, d, ret);
    }
    return kSuccess;
  }

  /// 零阶求值重载，分段选择和外推语义与带导数接口一致。
  ErrorType evaluate(const decimal_t s, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
    decimal_t h = s - vec_domain_[idx];

    if (s < vec_domain_[0]) {
      poly_[0].evaluate(h, ret);
    } else if (s > vec_domain_[num_pts - 1]) {
      h = s - vec_domain_[idx - 1];
      poly_[num_pts - 2].evaluate(h, ret);
    } else {
      poly_[idx].evaluate(h, ret);
    }
    return kSuccess;
  }

  /**
   * @brief 打印每段参数域、系数及段尾位置到 jerk，供二维样条调试。
   * @note 函数内部固定使用 Vecf<2>，仅适用于 N_DIM=2 的实例。
   */
  void print() const {
    int num_polys = static_cast<int>(poly_.size());
    for (int i = 0; i < num_polys; i++) {
      printf("vec domain (%lf, %lf).\n", vec_domain_[i], vec_domain_[i + 1]);
      poly_[i].print();
      Vecf<2> v;
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 0, &v);
      printf("end pos: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 1, &v);
      printf("end vel: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 2, &v);
      printf("end acc: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 3, &v);
      printf("end jerk: (%lf, %lf).\n", v[0], v[1]);
    }
  }

 private:
  /// 与相邻断点区间一一对应的局部向量多项式。
  vec_E<PolynomialType> poly_;
  /// 全局参数断点，预期严格递增但本类不验证。
  std::vector<decimal_t> vec_domain_;
};

}  // namespace common

#endif
