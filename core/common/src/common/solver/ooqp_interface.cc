#include "common/solver/ooqp_interface.h"

#include <stdexcept>

#include "ooqp/QpGenData.h"
#include "ooqp/QpGenVars.h"
#include "ooqp/QpGenResiduals.h"
#include "ooqp/GondzioSolver.h"
#include "ooqp/QpGenSparseMa27.h"
#include "ooqp/Status.h"

namespace common {

// 本实现沿用 OOQP C++ 接口命名，局部直接使用 Eigen 与标准库符号。
using namespace Eigen;
using namespace std;

// 将 Eigen 稀疏 QP 数据转换为 OOQP QpGenSparseMa27，求解后再复制主变量 x。
bool OoQpItf::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x, const bool ignoreUnknownError,
                    const bool verbose) {
  // 主变量数量由 Q 的行数决定；baseline 未验证 Q 为非空方阵或 c/l/u 维度一致。
  int nx = Q.rows();
  x.setZero(nx);

  // OOQP 接口可能修改输入缓存，因此复制线性项和约束数据。
  auto ccopy(c);
  auto Acopy(A);
  auto bcopy(b);
  auto Ccopy(C);

  // OOQP 只接收对称矩阵下三角；当前不验证原 Q 的对称性，所有上三角独有信息会丢失。
  SparseMatrix<double, Eigen::RowMajor> Q_triangular =
      Q.triangularView<Lower>();

  if (verbose) {
    printProblemFormulation(Q_triangular, ccopy, Acopy, bcopy, Ccopy, d, f, l,
                            u);
  }

  // OOQP 需要 CSR 风格连续数组，先压缩所有稀疏矩阵。
  Q_triangular.makeCompressed();
  Acopy.makeCompressed();
  Ccopy.makeCompressed();

  // 这里只检查不等式行数，其他矩阵/向量维度依赖调用方保证。
  assert(Ccopy.rows() == d.size());
  assert(Ccopy.rows() == f.size());
  // 将 +/-double_max 哨兵转换为 OOQP 的上下界启用标志。
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
  VectorXd lowerLimitForX;
  VectorXd upperLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForInequalityConstraints;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForInequalityConstraints;
  VectorXd lowerLimitForInequalityConstraints;
  VectorXd upperLimitForInequalityConstraints;

  generateLimits(l, u, useLowerLimitForX, useUpperLimitForX, lowerLimitForX,
                 upperLimitForX);
  generateLimits(d, f, useLowerLimitForInequalityConstraints,
                 useUpperLimitForInequalityConstraints,
                 lowerLimitForInequalityConstraints,
                 upperLimitForInequalityConstraints);
  if (verbose) {
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR X" << endl;
    printLimits(useLowerLimitForX, useUpperLimitForX, lowerLimitForX,
                upperLimitForX);
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR INEQUALITY CONSTRAINTS" << endl;
    printLimits(useLowerLimitForInequalityConstraints,
                useUpperLimitForInequalityConstraints,
                lowerLimitForInequalityConstraints,
                upperLimitForInequalityConstraints);
  }

  // 根据问题维数和非零元数量初始化 MA27 稀疏 QP 工厂。
  int my = bcopy.size();
  int mz = lowerLimitForInequalityConstraints.size();
  int nnzQ = Q_triangular.nonZeros();
  int nnzA = Acopy.nonZeros();
  int nnzC = Ccopy.nonZeros();

  QpGenSparseMa27* qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
  // 暴露 Eigen 内部连续数组给 OOQP。零长度向量仍调用 coeffRef(0)，是既有未定义行为。
  double* cp = &ccopy.coeffRef(0);
  int* krowQ = Q_triangular.outerIndexPtr();
  int* jcolQ = Q_triangular.innerIndexPtr();
  double* dQ = Q_triangular.valuePtr();
  double* xlow = &lowerLimitForX.coeffRef(0);
  char* ixlow = &useLowerLimitForX.coeffRef(0);
  double* xupp = &upperLimitForX.coeffRef(0);
  char* ixupp = &useUpperLimitForX.coeffRef(0);
  int* krowA = Acopy.outerIndexPtr();
  int* jcolA = Acopy.innerIndexPtr();
  double* dA = Acopy.valuePtr();
  double* bA = &bcopy.coeffRef(0);
  int* krowC = Ccopy.outerIndexPtr();
  int* jcolC = Ccopy.innerIndexPtr();
  double* dC = Ccopy.valuePtr();
  double* clow = &lowerLimitForInequalityConstraints.coeffRef(0);
  char* iclow = &useLowerLimitForInequalityConstraints.coeffRef(0);
  double* cupp = &upperLimitForInequalityConstraints.coeffRef(0);
  char* icupp = &useUpperLimitForInequalityConstraints.coeffRef(0);

  // makeData 不接管上述 Eigen 缓存所有权，缓存必须存活到求解结束。
  QpGenData* prob = (QpGenData*)qp->makeData(
      cp, krowQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, krowA, jcolA, dA, bA,
      krowC, jcolC, dC, clow, iclow, cupp, icupp);

  // 分别创建主/对偶变量、残差和 Gondzio 内点求解器。
  QpGenVars* vars = (QpGenVars*)qp->makeVariables(prob);
  QpGenResiduals* resid = (QpGenResiduals*)qp->makeResiduals(prob);
  GondzioSolver* s = new GondzioSolver(qp, prob);

  if (verbose) {
    s->monitorSelf();
  }

  // 求解并取得 OOQP 状态码。
  int status = s->solve(prob, vars, resid);

  // corridor 调用会允许 UNKNOWN，并把当时的迭代解复制为正式输出。
  if ((status == SUCCESSFUL_TERMINATION) ||
      (ignoreUnknownError && (status == UNKNOWN)))
    vars->x->copyIntoArray(&x.coeffRef(0));

  if (verbose) {
    printSolution(status, x);
  }
  // 依创建依赖的反序手工释放；若中途抛异常，这些裸指针不会自动清理。
  delete s;
  delete resid;
  delete vars;
  delete prob;
  delete qp;

  return ((status == SUCCESSFUL_TERMINATION) ||
          (ignoreUnknownError && (status == UNKNOWN)));
}

// 把极值哨兵界转为 OOQP 的启用标志；普通有限界保持原值并默认启用。
void OoQpItf::generateLimits(
    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit) {
  int n = l.size();
  useLowerLimit.setConstant(n, 1);
  useUpperLimit.setConstant(n, 1);
  lowerLimit = l;
  upperLimit = u;

  for (int i = 0; i < n; i++) {
    // 纯相对比较用于识别调用方传入的 -double_max 无界哨兵。
    if (NumericalUtil::ApproximatelyEqual(
            l(i), -std::numeric_limits<double>::max())) {
      useLowerLimit(i) = 0;
      lowerLimit(i) = 0.0;
    }
    // 正 double_max 同理表示不启用上界。
    if (NumericalUtil::ApproximatelyEqual(u(i),
                                          std::numeric_limits<double>::max())) {
      useUpperLimit(i) = 0;
      upperLimit(i) = 0.0;
    }
  }
}

// 将稀疏矩阵转为稠密文本打印，问题较大时会产生显著内存和日志开销。
void OoQpItf::printProblemFormulation(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u) {
  cout << "-------------------------------" << endl;
  cout << "Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and "
          "l <= x <= u"
       << endl
       << endl;
  cout << "Q (triangular) << " << endl << MatrixXd(Q) << endl;
  cout << "c << " << c.transpose() << endl;
  cout << "A << " << endl << MatrixXd(A) << endl;
  cout << "b << " << b.transpose() << endl;
  cout << "C << " << endl << MatrixXd(C) << endl;
  cout << "d << " << d.transpose() << endl;
  cout << "f << " << f.transpose() << endl;
  cout << "l << " << l.transpose() << endl;
  cout << "u << " << u.transpose() << endl;
}

// 打印每个分量是否启用上下界以及相应数值。
void OoQpItf::printLimits(
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit) {
  cout << "useLowerLimit << " << std::boolalpha
       << useLowerLimit.cast<bool>().transpose() << endl;
  cout << "lowerLimit << " << lowerLimit.transpose() << endl;
  cout << "useUpperLimit << " << std::boolalpha
       << useUpperLimit.cast<bool>().transpose() << endl;
  cout << "upperLimit << " << upperLimit.transpose() << endl;
}

// status==0 按成功格式打印；其他状态只输出错误码，不解释具体 OOQP 枚举。
void OoQpItf::printSolution(const int status, const Eigen::VectorXd& x) {
  if (status == 0) {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Ok, ended with status " << status << "." << endl;
    cout << "x << " << x.transpose() << endl;
  } else {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Error, ended with status " << status << "." << endl;
  }
}

}  // namespace common
