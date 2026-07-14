#include "common/solver/qp_solver.h"

#include <stdexcept>

#include "common/solver/ooqp_interface.h"

namespace common {

// 展开加权最小二乘目标并委托 OOQP 求解全部硬约束。
bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x) {
  // 原目标展开为 x'(A'SA+W)x-2x'A'Sb+常数。这里传入其一半，正比例缩放不改变
  // 仅含硬约束问题的最优解：OOQP 的 0.5*x'Qx+c'x 中 Q=A'SA+W、c=-A'Sb。
  int m = A.rows();
  int n = A.cols();
  // 求解失败时调用方看到维数正确的全零向量。
  x.setZero(n);
  // baseline 只用 assert 验证最小二乘三组核心维度，其他约束维度由下层隐式假设。
  assert(static_cast<int>(b.size()) == m);
  assert(static_cast<int>(S.rows()) == m);
  assert(static_cast<int>(W.rows()) == n);
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_temp;
  // W 本为对角矩阵，但当前先转稠密再 sparseView，规模大时会产生不必要内存开销。
  Q_temp = A.transpose() * S * A +
           (Eigen::SparseMatrix<double, Eigen::RowMajor>)W.toDenseMatrix()
               .sparseView();
  Eigen::VectorXd c_temp = -A.transpose() * S * b;
  return OoQpItf::solve(Q_temp, c_temp, C, c, D, d, f, l, u, x);
}

// 等式约束便捷入口：用接近 double 极值的边界表示变量无界，并传入零行 D。
bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c, Eigen::VectorXd& x) {
  // OoQpItf::generateLimits 会识别这两个极值哨兵并关闭相应变量界。
  int nx = A.cols();
  Eigen::VectorXd u =
      std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(nx);
  Eigen::VectorXd l = (-u.array()).matrix();
  // 空 D/d/f 表示没有双边线性不等式约束。
  Eigen::SparseMatrix<double, Eigen::RowMajor> D;
  Eigen::VectorXd d, f;
  return solve(A, S, b, W, C, c, D, d, f, l, u, x);
}

}  // namespace common
