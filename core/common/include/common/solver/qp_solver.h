#ifndef _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__
#define _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__

#include "common/basics/basics.h"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

namespace common {

/**
 * @brief 将正则化加权最小二乘问题转换为 OOQP 标准二次规划的适配器。
 *
 * 主要服务于样条系数拟合。实现参考 ooqp_eigen_interface，并把
 * `(Ax-b)'S(Ax-b)+x'Wx` 展开为二次项与线性项后交给 OoQpItf。
 */
class QuadraticProblem {
 public:
  /**
   * @brief 求解带等式、不等式和变量上下界的正则化加权最小二乘问题。
   *
   * 最小化 `(Ax-b)'S(Ax-b)+x'Wx`，满足 `Cx=c`、`d<=Dx<=f`、`l<=x<=u`。
   * @param A m x n 样本/模型矩阵。
   * @param S m x m 对角样本权重。
   * @param b m 维观测向量。
   * @param W n x n 对角变量正则权重。
   * @param C 等式约束矩阵，可为零行。
   * @param c 等式约束右端。
   * @param D 双边不等式约束矩阵，可为零行。
   * @param d 不等式下界。
   * @param f 不等式上界。
   * @param l 变量下界。
   * @param u 变量上界。
   * @param x 输出 n 维最优变量；进入函数时会先置零。
   * @return OOQP 报告成功时返回 true。
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x);
  /**
   * @brief 求解只有等式约束的重载。
   *
   * 内部构造无穷变量界和零行不等式矩阵，再委托完整入口。
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c, Eigen::VectorXd& x);
};

}  // namespace common

#endif
