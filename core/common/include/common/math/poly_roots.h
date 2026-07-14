/**
 * @file poly_roots.h
 * @brief 二至六次一元多项式的实根求解工具。
 *
 * 代码来源注明为 sikang/motion_primitive_library。二至四次使用闭式公式，五、六次
 * 使用 Eigen PolynomialSolver；所有接口只返回实现判定为纯实数的根，不排序也不
 * 去重。系数退化判断采用与零的精确比较，调用方需关注浮点尺度和病态多项式。
 */
#ifndef _CORE_COMMON_INC_COMMON_MATH_POLY_ROOTS_H__
#define _CORE_COMMON_INC_COMMON_MATH_POLY_ROOTS_H_
#include "common/basics/basics.h"
#include <unsupported/Eigen/Polynomials>
#include <iostream>

/**
 * @brief 求二次方程 b*t^2+c*t+d=0 的实根。
 * @note 直接调用时要求 b 非零；判别式为零时会返回两个相同的根。
 */
inline std::vector<decimal_t> quad(decimal_t b, decimal_t c, decimal_t d) {
  std::vector<decimal_t> dts;
  decimal_t p = c * c - 4 * b * d;
  if (p < 0)
    return dts;
  else {
    dts.push_back((-c - sqrt(p)) / (2 * b));
    dts.push_back((-c + sqrt(p)) / (2 * b));
    return dts;
  }
}

/**
 * @brief 使用 Cardano 公式求三次方程的实根。
 * @note 要求 a 非零，判别式分支使用精确零比较，返回根不去重。
 */
inline std::vector<decimal_t> cubic(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d) {
  std::vector<decimal_t> dts;

  decimal_t a2 = b / a;
  decimal_t a1 = c / a;
  decimal_t a0 = d / a;
  // Q、R 和 D 是降幂为首一三次式后的 Cardano 中间量。
  decimal_t Q = (3 * a1 - a2 * a2) / 9;
  decimal_t R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  decimal_t D = Q * Q * Q + R * R;
  if (D > 0) {
    // 一个实根和一对共轭复根，只返回实根。
    decimal_t S = std::cbrt(R + sqrt(D));
    decimal_t T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    // 存在重根；接口返回两个数值，不展开三重计数。
    decimal_t S = std::cbrt(R);
    dts.push_back(-a2/3+S+S);
    dts.push_back(-a2/3-S);
    return dts;
  }
  else {
    // 三个实根的三角形式；acos 输入未额外截断到 [-1, 1]。
    decimal_t theta = acos(R/sqrt(-Q*Q*Q));
    dts.push_back(2*sqrt(-Q)*cos(theta/3)-a2/3);
    dts.push_back(2*sqrt(-Q)*cos((theta+2*M_PI)/3)-a2/3);
    dts.push_back(2*sqrt(-Q)*cos((theta+4*M_PI)/3)-a2/3);
    return dts;
  }
}

/**
 * @brief 使用 Ferrari 型闭式分解求四次方程的实根。
 * @note 要求 a 非零；负根号产生的 NaN 会被过滤，输出不排序、不去重。
 */
inline std::vector<decimal_t> quartic(decimal_t a, decimal_t b, decimal_t c,
                                      decimal_t d, decimal_t e) {
  std::vector<decimal_t> dts;

  decimal_t a3 = b / a;
  decimal_t a2 = c / a;
  decimal_t a1 = d / a;
  decimal_t a0 = e / a;

  // 先求解预解三次方程，并使用其返回的第一个实根构造四次方程分解。
  std::vector<decimal_t> ys = cubic(1, -a2, a1*a3-4*a0, 4*a2*a0-a1*a1-a3*a3*a0);
  decimal_t y1 = ys.front();
  decimal_t r = a3*a3/4-a2+y1;

  // 当前根选择导致 r<0 时直接返回空集，不再尝试预解三次方程的其他根。
  if(r < 0)
    return dts;

  decimal_t R = sqrt(r);
  decimal_t D, E;
  if(R != 0) {
    D = sqrt(0.75*a3*a3-R*R-2*a2+0.25*(4*a3*a2-8*a1-a3*a3*a3)/R);
    E = sqrt(0.75*a3*a3-R*R-2*a2-0.25*(4*a3*a2-8*a1-a3*a3*a3)/R);
  }
  else {
    D = sqrt(0.75*a3*a3-2*a2+2*sqrt(y1*y1-4*a0));
    E = sqrt(0.75*a3*a3-2*a2-2*sqrt(y1*y1-4*a0));
  }

  // 仅以 NaN 判断根号分支是否有效；无穷值和重复根不会进一步处理。
  if(!std::isnan(D)) {
    dts.push_back(-a3/4+R/2+D/2);
    dts.push_back(-a3/4+R/2-D/2);
  }
  if(!std::isnan(E)) {
    dts.push_back(-a3/4-R/2+E/2);
    dts.push_back(-a3/4-R/2-E/2);
  }

  return dts;
}

/**
 * @brief 求最高四次多项式的实根，并按最高非零系数降阶。
 * @note a、b、c 可以为零；常数多项式统一返回空向量。
 */
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e) {
  std::vector<decimal_t> ts;
  if (a != 0)
    return quartic(a, b, c, d, e);
  else if (b != 0)
    return cubic(b, c, d, e);
  else if (c != 0)
    return quad(c, d, e);
  else if (d != 0) {
    ts.push_back(-e / d);
    return ts;
  } else
    return ts;
}

/**
 * @brief 求最高五次多项式的实根。
 *
 * 首项为零时委托四次入口；否则使用 Eigen 求全部复根，只保留虚部精确等于零者。
 */
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e, decimal_t f) {
  std::vector<decimal_t> ts;
  if (a == 0)
    return solve(b, c, d, e, f);
  else {
    Eigen::VectorXd coeff(6);
    coeff << f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 5> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 5>::RootsType &r = solver.roots();
    std::vector<decimal_t> ts;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}

/**
 * @brief 求最高六次多项式的实根。
 *
 * 非退化时使用 Eigen 六次求解器并仅接收虚部精确为零的根。baseline 只有 a 和 b
 * 同时为零才降到四次入口，a=0、b!=0 的五次退化情形仍会进入六次求解器。
 */
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e, decimal_t f,
                                    decimal_t g) {
  std::vector<decimal_t> ts;
  if (a == 0 && b == 0)
    return solve(c, d, e, f, g);
  else {
    Eigen::VectorXd coeff(7);
    coeff << g, f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 6> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 6>::RootsType &r = solver.roots();
    std::vector<decimal_t> ts;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}

#endif
