#include "common/lane/lane.h"

namespace common {

// 使用 r'(s)、r''(s)、r'''(s) 计算二维有符号曲率及实现定义的曲率导数。
// 注意：这里沿用 baseline 公式和运算优先级，不在纯注释任务中调整数值表达式。
ErrorType Lane::GetCurvatureByArcLength(const decimal_t& arc_length,
                                        decimal_t* curvature,
                                        decimal_t* curvature_derivative) const {
  if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
    return kIllegalInput;
  }

  // vel/acc/jrk 分别是位置样条对参数 s 的一、二、三阶导数；名称不表示时间量。
  Vecf<LaneDim> vel, acc, jrk;
  GetDerivativeByArcLength(arc_length, 1, &vel);
  GetDerivativeByArcLength(arc_length, 2, &acc);
  GetDerivativeByArcLength(arc_length, 3, &jrk);

  // 二维叉积给出曲率符号；分母使用一阶导数模长的三次方。
  decimal_t c0 = vel[0] * acc[1] - vel[1] * acc[0];
  decimal_t c1 = vel.norm();
  *curvature = c0 / (c1 * c1 * c1);
  *curvature_derivative =
      ((acc[0] * acc[1] + vel[0] * jrk[1] - acc[1] * acc[0] - vel[1] * jrk[0]) /
           c1 * c1 * c1 -
       3 * c0 * (vel[0] * acc[0] + vel[1] * acc[1]) / (c1 * c1 * c1 * c1 * c1));
  return kSuccess;
}

// 只计算曲率的轻量重载；仍需两次样条导数求值。
ErrorType Lane::GetCurvatureByArcLength(const decimal_t& arc_length,
                                        decimal_t* curvature) const {
  if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel, acc;
  GetDerivativeByArcLength(arc_length, 1, &vel);
  GetDerivativeByArcLength(arc_length, 2, &acc);

  decimal_t c0 = vel[0] * acc[1] - vel[1] * acc[0];
  decimal_t c1 = vel.norm();
  *curvature = c0 / (c1 * c1 * c1);
  return kSuccess;
}

// 本层不重复做范围检查，统一透传底层样条的求值结果。
ErrorType Lane::GetDerivativeByArcLength(const decimal_t arc_length,
                                         const int d,
                                         Vecf<LaneDim>* derivative) const {
  return position_spline_.evaluate(arc_length, d, derivative);
}

// d=0 的位置求值便捷接口。
ErrorType Lane::GetPositionByArcLength(const decimal_t arc_length,
                                       Vecf<LaneDim>* derivative) const {
  return position_spline_.evaluate(arc_length, derivative);
}

ErrorType Lane::GetTangentVectorByArcLength(
    const decimal_t arc_length, Vecf<LaneDim>* tangent_vector) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  // 对一阶导数归一化。退化点无法定义稳定切向，因此显式拒绝。
  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  *tangent_vector = vel / vel.norm();
  return kSuccess;
}

ErrorType Lane::GetNormalVectorByArcLength(const decimal_t arc_length,
                                           Vecf<LaneDim>* normal_vector) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  // 项目约定左法向为切向逆时针旋转 90 度的结果。
  Vecf<LaneDim> tangent_vector = vel / vel.norm();
  *normal_vector = rotate_vector_2d(tangent_vector, M_PI / 2.0);
  return kSuccess;
}

ErrorType Lane::GetOrientationByArcLength(const decimal_t arc_length,
                                          decimal_t* angle) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  // 航向只由单位切向决定，不读取车辆姿态或道路拓扑方向。
  Vecf<LaneDim> tangent_vector = vel / vel.norm();
  *angle = vec2d_to_angle(tangent_vector);
  return kSuccess;
}

ErrorType Lane::GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
                                          decimal_t* arc_length) const {
  if (!IsValid()) {
    return kWrongStatus;
  }

  // 粗搜索最多执行 4 次；任一候选点进入 30 m 半径后便接受当前最近候选为初值。
  static constexpr int kMaxCnt = 4;
  static constexpr decimal_t kMaxDistSquare = 900.0;

  // 初始三点覆盖整个样条参数域：下界、中点、上界。
  const decimal_t val_lb = position_spline_.begin();
  const decimal_t val_ub = position_spline_.end();
  decimal_t step = (val_ub - val_lb) * 0.5;

  decimal_t s1 = val_lb;
  decimal_t s2 = val_lb + step;
  decimal_t s3 = val_ub;
  decimal_t initial_guess = s2;

  Vecf<LaneDim> start_pos, mid_pos, final_pos;
  position_spline_.evaluate(s1, &start_pos);
  position_spline_.evaluate(s2, &mid_pos);
  position_spline_.evaluate(s3, &final_pos);

  // 阶段一：比较三点到查询点的距离平方，并围绕当前最优点将步长减半。
  // 这是一种有限次数的三点区间缩小策略，并非严格的二分最近点搜索。
  decimal_t d1 = (start_pos - vec_position).squaredNorm();
  decimal_t d2 = (mid_pos - vec_position).squaredNorm();
  decimal_t d3 = (final_pos - vec_position).squaredNorm();

  for (int i = 0; i < kMaxCnt; ++i) {
    decimal_t min_dis = std::min(std::min(d1, d2), d3);
    if (min_dis < kMaxDistSquare) {
      if (min_dis == d1) {
        initial_guess = s1;
      } else if (min_dis == d2) {
        initial_guess = s2;
      } else if (min_dis == d3) {
        initial_guess = s3;
      } else {
        assert(false);
      }
      break;
    }
    // 所有候选均在 30 m 之外时才继续缩小搜索区间。
    step *= 0.5;
    if (min_dis == d1) {
      initial_guess = s1;
      s3 = s2;
      s2 = s1 + step;
      position_spline_.evaluate(s2, &mid_pos);
      position_spline_.evaluate(s3, &final_pos);
      d2 = (mid_pos - vec_position).squaredNorm();
      d3 = (final_pos - vec_position).squaredNorm();
    } else if (min_dis == d2) {
      initial_guess = s2;
      s1 = s2 - step;
      s3 = s2 + step;
      position_spline_.evaluate(s1, &start_pos);
      position_spline_.evaluate(s3, &final_pos);
      d1 = (start_pos - vec_position).squaredNorm();
      d3 = (final_pos - vec_position).squaredNorm();
    } else if (min_dis == d3) {
      initial_guess = s3;
      s1 = s2;
      s2 = s3 - step;
      position_spline_.evaluate(s1, &start_pos);
      position_spline_.evaluate(s2, &mid_pos);
      d1 = (start_pos - vec_position).squaredNorm();
      d2 = (mid_pos - vec_position).squaredNorm();
    } else {
      printf(
          "[Lane]GetArcLengthByVecPosition - d1: %lf, d2: %lf, d3: %lf, "
          "min_dis: %lf\n",
          d1, d2, d3, min_dis);
      assert(false);
    }
  }

  // 阶段二：以粗搜索结果为初值，局部最小化点到曲线的距离平方。
  // baseline 忽略下层错误码并固定返回成功，后续鲁棒性修复需单独改变该契约。
  GetArcLengthByVecPositionWithInitialGuess(vec_position, initial_guess,
                                            arc_length);

  return kSuccess;
}

ErrorType Lane::GetArcLengthByVecPositionWithInitialGuess(
    const Vecf<LaneDim>& vec_position, const decimal_t& initial_guess,
    decimal_t* arc_length) const {
  if (!IsValid()) {
    return kWrongStatus;
  }

  const decimal_t val_lb = position_spline_.begin();
  const decimal_t val_ub = position_spline_.end();

  // 对 1/2*||r(s)-q||^2 做 Newton 迭代，收敛阈值为 1e-3，最多 8 次。
  static constexpr decimal_t epsilon = 1e-3;
  static constexpr int kMaxIter = 8;
  decimal_t x = std::min(std::max(initial_guess, val_lb), val_ub);
  Vecf<LaneDim> p, dp, ddp, tmp_vec;

  for (int i = 0; i < kMaxIter; ++i) {
    position_spline_.evaluate(x, 0, &p);
    position_spline_.evaluate(x, 1, &dp);
    position_spline_.evaluate(x, 2, &ddp);

    // 一阶条件 f_1=(r-q)^T r'；二阶项 f_2=r'^T r'+(r-q)^T r''。
    // 当前实现不检查 f_2 是否接近零，也不验证最终点是否为局部极小值。
    tmp_vec = p - vec_position;
    double f_1 = tmp_vec.dot(dp);
    double f_2 = dp.dot(dp) + tmp_vec.dot(ddp);
    double dx = -f_1 / f_2;

    if (std::fabs(dx) < epsilon) {
      break;
    }

    // 一旦迭代越界，直接采用相应边界并结束，不执行线搜索或回溯。
    if (x + dx > val_ub) {
      x = val_ub;
      break;
    } else if (x + dx < val_lb) {
      x = val_lb;
      break;
    }

    x += dx;
  }

  *arc_length = x;

  return kSuccess;
}

// 历史实现：先在样条节点上找最近点，再以固定 0.05 m 参数步长双向搜索。
// 当前未参与编译，保留仅用于理解投影算法的演进过程。
// ErrorType Lane::GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
//                                           decimal_t* arc_length) const {
//   if (!IsValid()) {
//     return kWrongStatus;
//   }

//   std::vector<decimal_t> vec_domain = position_spline_.vec_domain();
//   int num_pts = static_cast<int>(vec_domain.size());

//   decimal_t min_dis = kInf;
//   decimal_t min_s = 0.0;
//   int idx = 0;
//   decimal_t dis;
//   for (int i = 0; i < num_pts; i++) {
//     Vecf<LaneDim> vec{Vecf<LaneDim>::Zero()};
//     position_spline_.evaluate(vec_domain[i], &vec);
//     dis = (vec - vec_position).norm();
//     if (dis < min_dis) {
//       idx = i;
//       min_dis = dis;
//       min_s = vec_domain[i];
//     }
//   }

//   int low_idx = std::max(0, idx - 1);
//   int up_idx = std::min(idx + 1, num_pts - 1);

//   const decimal_t step = 0.05;  // 0.05-->2cm error, 0.01-->less than 1 cm
//   error

//   // ~ search forward
//   decimal_t min_dis_forward = min_dis;
//   decimal_t min_s_forward = min_s;
//   for (decimal_t s = vec_domain[idx]; s < vec_domain[up_idx]; s += step) {
//     Vecf<LaneDim> vec{Vecf<LaneDim>::Zero()};
//     position_spline_.evaluate(s, &vec);
//     dis = (vec - vec_position).norm();
//     if (dis < min_dis_forward + kEPS) {
//       min_dis_forward = dis;
//       min_s_forward = s;
//     } else {
//       break;
//     }
//   }

//   decimal_t min_dis_backward = min_dis;
//   decimal_t min_s_backward = min_s;
//   for (decimal_t s = vec_domain[idx]; s > vec_domain[low_idx]; s -= step) {
//     Vecf<LaneDim> vec{Vecf<LaneDim>::Zero()};
//     position_spline_.evaluate(s, &vec);
//     dis = (vec - vec_position).norm();
//     if (dis < min_dis_backward + kEPS) {
//       min_dis_backward = dis;
//       min_s_backward = s;
//     } else {
//       // not make progress any more, break
//       break;
//     }
//   }

//   *arc_length =
//       min_dis_forward < min_dis_backward ? min_s_forward : min_s_backward;
//   return kSuccess;
// }

ErrorType Lane::CheckInputArcLength(const decimal_t arc_length) const {
  if (!IsValid()) {
    printf("[CheckInputArcLength]Quering invalid lane.\n");
    return kWrongStatus;
  }
  // 边界检查允许 kEPS 数值容差；是否能在容差外延处求值仍由底层样条决定。
  if (arc_length < position_spline_.begin() - kEPS ||
      arc_length > position_spline_.end() + kEPS) {
    // printf("[CheckInputArcLength]Out of domain error: %lf --> (%lf, %lf).\n",
    //        arc_length, position_spline_.begin(), position_spline_.end());
    return kIllegalInput;
  }
  return kSuccess;
}

}  // namespace common
