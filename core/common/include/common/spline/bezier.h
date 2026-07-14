#ifndef _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__

#include <assert.h>

#include <vector>

namespace common {

// 前置声明：BezierSpline::evaluate 在模板实例化时调用其静态基函数接口。
template <int N_DEG>
class BezierUtils;

/**
 * @brief 采用时间缩放 Bernstein 基的固定阶数、固定维数分段 Bezier 样条。
 *
 * 第 j 段以 duration_j=t_{j+1}-t_j 归一化局部参数 tau=(t-t_j)/duration_j，
 * d 阶导数按 duration_j^(1-d) 缩放。因零阶还乘一个 duration，ctrl_pts_ 保存的是
 * 项目优化变量而非通常意义下可直接解释为世界位置的几何控制点。
 */
template <int N_DEG, int N_DIM>
class BezierSpline {
 public:
  /// 构造空参数域、空控制变量数组。
  BezierSpline() {}

  /**
   * @brief 设置全局参数断点，并为每个分段分配全零控制变量矩阵。
   * @param vec_domain 输入断点；baseline 只断言数量大于 1。
   */
  void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
    assert(vec_domain.size() > 1);
    vec_domain_ = vec_domain;
    ctrl_pts_.resize(vec_domain.size() - 1);
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      ctrl_pts_[j].setZero();
    }
  }

  /// 写入指定分段、指定 Bernstein 下标对应的 N_DIM 维控制变量。
  void set_coeff(const int segment_idx, const int ctrl_pt_index,
                 const Vecf<N_DIM>& coeff) {
    ctrl_pts_[segment_idx].row(ctrl_pt_index) = coeff;
  }

  /// 整体替换控制变量矩阵数组，不检查其段数是否与 vec_domain_ 一致。
  void set_ctrl_pts(const vec_E<Matf<N_DEG + 1, N_DIM>>& pts) {
    ctrl_pts_ = pts;
  }

  /// 返回当前控制变量矩阵数量，即实现认定的分段数。
  int num_segments() const { return static_cast<int>(ctrl_pts_.size()); }

  /// 返回全局参数断点副本。
  std::vector<decimal_t> vec_domain() const { return vec_domain_; }

  /// 返回所有分段控制变量矩阵的副本。
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts() const { return ctrl_pts_; }

  /// 返回参数域起点；空对象返回 0。
  decimal_t begin() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.front();
  }

  /// 返回参数域终点；空对象返回 0。
  decimal_t end() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.back();
  }

  /**
   * @brief 查询全局参数 s 处的 d 阶导数向量。
   * @param s 全局参数。
   * @param d 导数阶数；五次实现只定义 0 至 3 阶基函数。
   * @param ret 输出向量。
   * @note 左越界返回 kIllegalInput；右越界通过基函数截断到末点并返回成功。
   */
  ErrorType evaluate(const decimal_t s, const int d, Vecf<N_DIM>* ret) const {
    // 与普通 Spline 一样，只拒绝完全空域，单断点异常对象仍可能越界访问。
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    // 精确命中内部断点时选择左段；右越界时把分段下标截断到最后一段。
    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx =
        std::min(std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0),
                 num_pts - 2);
    decimal_t h = s - vec_domain_[idx];
    if (s < vec_domain_[0]) {
      return kIllegalInput;
    } else {
      // duration 必须为正；零宽或逆序断点会导致归一化和尺度因子失效。
      decimal_t duration = vec_domain_[idx + 1] - vec_domain_[idx];
      decimal_t normalized_s = h / duration;
      // GetBezierBasis 会把 normalized_s 截断到 [0,1]，因此右越界等价于末点查询。
      Vecf<N_DEG + 1> basis =
          BezierUtils<N_DEG>::GetBezierBasis(d, normalized_s);
      Vecf<N_DIM> result =
          (pow(duration, 1 - d) * basis.transpose() * ctrl_pts_[idx])
              .transpose();
      *ret = result;
    }
    return kSuccess;
  }

  /// 打印各分段控制变量矩阵，供优化结果调试。
  void print() const {
    printf("Bezier control points.\n");
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      printf("segment %d -->.\n", j);
      std::cout << ctrl_pts_[j] << std::endl;
    }
  }

 private:
  /// 每段一个 (N_DEG+1)xN_DIM 的时间缩放 Bernstein 控制变量矩阵。
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts_;
  /// 全局参数断点，预期严格递增但本类不验证。
  std::vector<decimal_t> vec_domain_;
};

/**
 * @brief 五次 Bernstein 基函数、导数基函数及 jerk 代价 Hessian 的静态工具。
 * @note 当前常量表达式只实现 N_DEG=5。
 */
template <int N_DEG>
class BezierUtils {
 public:
  /**
   * @brief 返回未做分段时长缩放的 Bernstein 控制变量代价 Hessian。
   * @param derivative_degree 代价所用导数阶数；当前只支持 jerk，即 3。
   */
  static MatNf<N_DEG + 1> GetBezierHessianMat(int derivative_degree) {
    MatNf<N_DEG + 1> hessian;
    switch (N_DEG) {
      case 5: {
        if (derivative_degree == 3) {
          // 五次 Bezier 曲线三阶导数平方积分对应的固定对称矩阵。
          hessian << 720.0, -1800.0, 1200.0, 0.0, 0.0, -120.0, -1800.0, 4800.0,
              -3600.0, 0.0, 600.0, 0.0, 1200.0, -3600.0, 3600.0, -1200.0, 0.0,
              0.0, 0.0, 0.0, -1200.0, 3600.0, -3600.0, 1200.0, 0.0, 600.0, 0.0,
              -3600.0, 4800.0, -1800.0, -120.0, 0.0, 0.0, 1200.0, -1800.0,
              720.0;
          break;
        } else {
          assert(false);
        }
        break;
      }
      default:
        assert(false);
    }
    return hessian;
  }

  /**
   * @brief 返回未做分段时长缩放的 Bernstein 基或其导数基向量。
   * @param derivative_degree 导数阶数；五次实现支持 0、1、2、3。
   * @param t 归一化局部参数，函数会先截断到 [0,1]。
   */
  static Vecf<N_DEG + 1> GetBezierBasis(int derivative_degree, decimal_t t) {
    // 基函数工具本身采用饱和语义，不报告归一化参数越界。
    t = std::max(std::min(1.0, t), 0.0);
    Vecf<N_DEG + 1> basis;
    switch (N_DEG) {
      case 5:
        if (derivative_degree == 0) {
          basis << -pow(t - 1, 5), 5 * t * pow(t - 1, 4),
              -10 * pow(t, 2) * pow(t - 1, 3), 10 * pow(t, 3) * pow(t - 1, 2),
              -5 * pow(t, 4) * (t - 1), pow(t, 5);
        } else if (derivative_degree == 1) {
          basis << -5 * pow(t - 1, 4),
              20 * t * pow(t - 1, 3) + 5 * pow(t - 1, 4),
              -20 * t * pow(t - 1, 3) - 30 * pow(t, 2) * pow(t - 1, 2),
              10 * pow(t, 3) * (2 * t - 2) + 30 * pow(t, 2) * pow(t - 1, 2),
              -20 * pow(t, 3) * (t - 1) - 5 * pow(t, 4), 5 * pow(t, 4);
        } else if (derivative_degree == 2) {
          basis << -20 * pow(t - 1, 3),
              60 * t * pow(t - 1, 2) + 40 * pow(t - 1, 3),
              -120 * t * pow(t - 1, 2) - 20 * pow(t - 1, 3) -
                  30 * t * t * (2 * t - 2),
              60 * t * pow(t - 1, 2) + 60 * t * t * (2 * t - 2) +
                  20 * t * t * t,
              -60 * t * t * (t - 1) - 40 * t * t * t, 20 * t * t * t;
        } else if (derivative_degree == 3) {
          basis << -60 * pow(t - 1, 2),
              60 * t * (2 * t - 2) + 180 * pow(t - 1, 2),
              -180 * t * (2 * t - 2) - 180 * pow(t - 1, 2) - 60 * t * t,
              180 * t * (2 * t - 2) + 60 * pow(t - 1, 2) + 180 * t * t,
              -120 * t * (t - 1) - 180 * t * t, 60 * t * t;
        }
        // 五次但导数阶数不在 0..3 时没有错误分支，basis 保持未初始化。
        break;
      default:
        // 其他阶数没有常量实现；关闭 assert 后 basis 仍未初始化。
        printf("N_DEG %d, derivative_degree %d.\n", N_DEG, derivative_degree);
        assert(false);
        break;
    }
    return basis;
  }
};

}  // namespace common

#endif
