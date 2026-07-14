#ifndef _CORE_COMMON_INC_COMMON_SOLVER_OOQP_INTERFACE_H__
#define _CORE_COMMON_INC_COMMON_SOLVER_OOQP_INTERFACE_H__

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

#include <algorithm>  // std::max
#include <cmath>      // std::abs
#include <limits>     // std::numeric_limits

namespace common {

/**
 * @brief 把 Eigen 稀疏矩阵/向量转换为 OOQP QpGenSparseMa27 数据结构的静态接口。
 *
 * 求解标准凸 QP：`min 0.5*x'Qx+c'x`，满足等式、双边线性不等式和变量上下界。
 * 调用方负责矩阵对称半正定、维度一致、数值有限和上下界有序。
 */
class OoQpItf {
 public:
  /**
   * @brief 求解 OOQP 标准二次规划。
   * @param Q n x n 对称半正定二次矩阵；实现只传递其下三角。
   * @param c n 维线性目标。
   * @param A 等式约束矩阵，可为零行。
   * @param b 等式约束右端。
   * @param C 双边线性不等式矩阵，可为零行。
   * @param d 不等式下界。
   * @param f 不等式上界。
   * @param l n 维变量下界；`-double_max` 被解释为未启用。
   * @param u n 维变量上界；`double_max` 被解释为未启用。
   * @param x 输出解，进入函数时按 Q 行数清零。
   * @param ignoreUnknownError 为 true 时把 OOQP UNKNOWN 状态也当作成功并复制当前解。
   * @param verbose 是否打印完整问题、边界启用标志、状态和解。
   * @return 成功终止，或允许 UNKNOWN 且状态为 UNKNOWN 时返回 true。
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x, const bool ignoreUnknownError = false,
                    const bool verbose = false);

 private:
  /**
   * @brief 将极值哨兵上下界转换为 OOQP 的“是否启用界”标志与数值数组。
   */
  static void generateLimits(
      const Eigen::VectorXd& l, const Eigen::VectorXd& u,
      Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
      Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
      Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit);

  /// 以稠密文本形式打印 QP 全部矩阵与向量，仅用于调试小问题。
  static void printProblemFormulation(
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
      const Eigen::VectorXd& c,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
      const Eigen::VectorXd& b,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
      const Eigen::VectorXd& d, const Eigen::VectorXd& f,
      const Eigen::VectorXd& l, const Eigen::VectorXd& u);

  /// 打印上下界启用标志及传给 OOQP 的实际数值。
  static void printLimits(
      const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
      const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
      const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit);

  /// 按 OOQP 状态码打印成功解或错误信息。
  static void printSolution(const int status, const Eigen::VectorXd& x);
};

/**
 * @brief 求解器边界哨兵识别使用的相对浮点比较工具。
 */
class NumericalUtil {
 public:
  /**
   * @brief 返回 `max(|a|,|b|)*epsilon` 作为相对误差尺度。
   */
  template <typename ValueType_>
  static inline ValueType_ maxTimesEpsilon(const ValueType_ a,
                                           const ValueType_ b,
                                           const ValueType_ epsilon) {
    return std::max(std::abs(a), std::abs(b)) * epsilon;
  }
  /**
   * @brief 判断两数之差是否不超过纯相对误差阈值。
   * @note 没有绝对误差下限，因此接近零时近似等价于精确比较。
   */
  template <typename ValueType_>
  static bool ApproximatelyEqual(
      const ValueType_ a, const ValueType_ b,
      ValueType_ epsilon = std::numeric_limits<ValueType_>::epsilon()) {
    return std::abs(a - b) <= maxTimesEpsilon(a, b, epsilon);
  }
};

}  // namespace common

#endif
