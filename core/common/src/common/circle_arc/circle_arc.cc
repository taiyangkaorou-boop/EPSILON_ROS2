#include "common/circle_arc/circle_arc.h"

namespace common {

// 根据曲率是否精确为零，分别初始化圆弧或直线的解析传播参数。
CircleArc::CircleArc(const Vec3f &start_state, const double &curvature,
                     const double &arc_length)
    : start_state_(start_state),
      curvature_(curvature),
      arc_length_(arc_length) {
  if (curvature_ != 0.0) {
    is_arc_ = true;

    // 有符号半径 r=1/kappa；负曲率自然得到右转圆弧。
    central_angle_ = curvature_ * arc_length_;
    double r = 1.0 / curvature_;

    // 圆心位于起始航向左法向的 r 倍位置。
    center_(0) = start_state_(0) - r * sin(start_state_(2));
    center_(1) = start_state_(1) + r * cos(start_state_(2));

    // 沿圆周转过 central_angle_，末航向不执行角度归一化。
    final_state_(0) = center_(0) + r * sin(start_state_(2) + central_angle_);
    final_state_(1) = center_(1) - r * cos(start_state_(2) + central_angle_);
    final_state_(2) = start_state_(2) + central_angle_;

  } else {
    is_arc_ = false;
    // 零曲率退化为沿起始航向传播 arc_length_ 的直线段。
    final_state_(0) = start_state_(0) + arc_length_ * cos(start_state_(2));
    final_state_(1) = start_state_(1) + arc_length_ * sin(start_state_(2));
    final_state_(2) = start_state_(2);
  }
}

// 圆弧分支按圆心和半径求 x；直线分支按起点和航向线性传播。
double CircleArc::x_d0(const double s) const {
  if (is_arc_) {
    double r = 1 / curvature_;
    return center_(0) + r * sin(start_state_(2) + s / r);
  } else {
    return start_state_(0) + s * cos(start_state_(2));
  }
}

// x 对弧长参数的一阶导数即单位切向的 x 分量。
double CircleArc::x_d1(const double s) const {
  if (is_arc_) {
    return cos(start_state_(2) + s * curvature_);
  } else {
    return cos(start_state_(2));
  }
}

// x 对弧长参数的二阶导数等于曲率乘单位左法向的 x 分量。
double CircleArc::x_d2(const double s) const {
  if (is_arc_) {
    return -curvature_ * sin(start_state_(2) + s * curvature_);
  } else {
    return 0;
  }
}

// y 坐标的解析传播，与 x_d0 使用同一圆心和有符号半径。
double CircleArc::y_d0(const double s) const {
  if (is_arc_) {
    double r = 1 / curvature_;
    return center_(1) - r * cos(start_state_(2) + s / r);
  } else {
    return start_state_(1) + s * sin(start_state_(2));
  }
}

// y 对弧长参数的一阶导数即单位切向的 y 分量。
double CircleArc::y_d1(const double s) const {
  if (is_arc_) {
    return sin(start_state_(2) + s * curvature_);
  } else {
    return sin(start_state_(2));
  }
}

// y 对弧长参数的二阶导数等于曲率乘单位左法向的 y 分量。
double CircleArc::y_d2(const double s) const {
  if (is_arc_) {
    return curvature_ * cos(start_state_(2) + s * curvature_);
  } else {
    return 0;
  }
}

// 恒曲率模型下航向随弧长线性变化；直线航向保持不变。
double CircleArc::theta_d0(const double s) const {
  if (is_arc_) {
    return start_state_(2) + s * curvature_;
  } else {
    return start_state_(2);
  }
}

// 航向关于弧长的一阶导数就是曲率。
double CircleArc::theta_d1(const double s) const {
  if (is_arc_) {
    return curvature_;
  } else {
    return 0;
  }
}

// 在中心线位置上叠加 offs 倍左单位法向，正 offs 指向车辆左侧。
double CircleArc::x_offs_d0(const double s, const double offs) const {
  return x_d0(s) + offs * nx_d0(s);
}

// y 分量使用同一左法向偏移约定。
double CircleArc::y_offs_d0(const double s, const double offs) const {
  return y_d0(s) + offs * ny_d0(s);
}

// 平行偏移曲线沿用中心线切向方向；奇异偏移半径未单独处理。
double CircleArc::theta_offs_d0(const double s, const double offs) const {
  return theta_d0(s);
}

// 按 s_step 遍历弧长参数并解析计算状态，不缓存采样结果。
void CircleArc::GetSampledStates(const double s_step,
                                 std::vector<Vec3f> *p_sampled_states) const {
  // 输出采用追加语义；循环只采到 |s|<|arc_length_|，通常不包含精确末点。
  for (double length = 0.0; fabs(length) < fabs(arc_length_);
       length += s_step) {
    Vec3f sampled_state;
    sampled_state(0) = x_d0(length);
    sampled_state(1) = y_d0(length);
    sampled_state(2) = theta_d0(length);
    p_sampled_states->emplace_back(sampled_state);
  }
}

}  // namespace common
