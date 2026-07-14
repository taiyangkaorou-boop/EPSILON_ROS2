#include "common/primitive/frenet_primitive.h"

namespace common {

// 用两端二阶边界分别构造 s 多项式和 d 时间/弧长多项式。
ErrorType FrenetPrimitive::Connect(const FrenetState& fs0,
                                   const FrenetState& fs1,
                                   const decimal_t stamp, const decimal_t T,
                                   bool is_lateral_independent) {
  is_lateral_independent_ = is_lateral_independent;

  // 纵向始终按给定持续时间 T 连接 s、s_dot、s_ddot。
  poly_s_.GetJerkOptimalConnection(fs0.vec_s(0), fs0.vec_s(1), fs0.vec_s(2),
                                   fs1.vec_s(0), fs1.vec_s(1), fs1.vec_s(2), T);

  if (is_lateral_independent) {
    // 高速/时间独立模式直接连接 d 对时间的一、二阶导数。
    poly_d_.GetJerkOptimalConnection(fs0.vec_dt(0), fs0.vec_dt(1),
                                     fs0.vec_dt(2), fs1.vec_dt(0),
                                     fs1.vec_dt(1), fs1.vec_dt(2), T);
  } else {
    // 弧长模式连接 d 对 s 的导数；纵向位移过小时用 100 m 虚拟跨度避免奇异。
    if (fs1.vec_s[0] - fs0.vec_s[0] < kSmallDistanceThreshold_) {
      const decimal_t virtual_large_distance = 100.0;
      poly_d_.GetJerkOptimalConnection(
          fs0.vec_ds(0), fs0.vec_ds(1), fs0.vec_ds(2), fs1.vec_ds(0),
          fs1.vec_ds(1), fs1.vec_ds(2), virtual_large_distance);
    } else {
      poly_d_.GetJerkOptimalConnection(
          fs0.vec_ds(0), fs0.vec_ds(1), fs0.vec_ds(2), fs1.vec_ds(0),
          fs1.vec_ds(1), fs1.vec_ds(2), fs1.vec_s[0] - fs0.vec_s[0]);
    }
  }

  stamp_ = stamp;
  fs0_ = fs0;
  duration_ = T;
  fs1_ = fs1;
  return kSuccess;
}

// 以常加速度二次多项式传播时间独立 Frenet 状态，并缓存终点。
ErrorType FrenetPrimitive::Propagate(const FrenetState& fs0, const Vecf<2>& u,
                                     const decimal_t stamp, const decimal_t T) {
  is_lateral_independent_ = true;
  Vecf<6> coeff;
  // Polynomial 的阶乘缩放布局使二阶导数 u 位于索引 3。
  coeff << 0.0, 0.0, 0.0, u[0], fs0.vec_s[1], fs0.vec_s[0];
  poly_s_.set_coeff(coeff);
  coeff << 0.0, 0.0, 0.0, u[1], fs0.vec_dt[1], fs0.vec_dt[0];
  poly_d_.set_coeff(coeff);
  duration_ = T;
  stamp_ = stamp;
  fs0_ = fs0;
  // baseline 忽略终点查询错误；T<=kEPS 时 fs1_ 可能保持旧值/默认值。
  GetFrenetState(stamp + T, &fs1_);
  return kSuccess;
}

// 把全局时间转换为局部 t，并按横向参数化模式加载 FrenetState。
ErrorType FrenetPrimitive::GetFrenetState(const decimal_t t_global,
                                          FrenetState* fs) const {
  if (duration_ < kEPS) return kWrongStatus;
  auto t = t_global - stamp_;
  if (is_lateral_independent_) {
    // s(t) 与 d(t) 均直接计算到二阶时间导数。
    fs->Load(Vecf<3>(poly_s_.evaluate(t, 0), poly_s_.evaluate(t, 1),
                     poly_s_.evaluate(t, 2)),
             Vecf<3>(poly_d_.evaluate(t, 0), poly_d_.evaluate(t, 1),
                     poly_d_.evaluate(t, 2)),
             FrenetState::kInitWithDt);
    fs->time_stamp = t_global;
  } else {
    // 先由时间得到纵向位置，再用相对纵向位移查询 d(delta_s)。
    auto s = poly_s_.evaluate(t, 0);
    auto st = s - fs0_.vec_s[0];
    fs->Load(Vecf<3>(s, poly_s_.evaluate(t, 1), poly_s_.evaluate(t, 2)),
             Vecf<3>(poly_d_.evaluate(st, 0), poly_d_.evaluate(st, 1),
                     poly_d_.evaluate(st, 2)),
             FrenetState::kInitWithDs);
    fs->time_stamp = t_global;
  }
  return kSuccess;
}

// 固定步长采样半开区间 `[begin+offset,end)`，失败样本被静默跳过。
ErrorType FrenetPrimitive::GetFrenetStateSamples(
    const decimal_t step, const decimal_t offset,
    vec_E<FrenetState>* fs_vec) const {
  FrenetState fs;
  fs_vec->clear();
  // reserve 估计和循环都假设 step>0；零或负步长未验证。
  int num_samples_esti =
      static_cast<int>((end() - begin() - offset) / step) + 10;
  fs_vec->reserve(num_samples_esti);
  for (decimal_t t = begin() + offset; t < end(); t += step) {
    if (GetFrenetState(t, &fs) == kSuccess) {
      fs_vec->push_back(fs);
    }
  }
  return kSuccess;
}

// 计算纵向与横向多项式三阶导数平方积分。
ErrorType FrenetPrimitive::GetJ(decimal_t* c_s, decimal_t* c_d) const {
  *c_s = poly_s_.J(duration_, 3);
  if (is_lateral_independent_) {
    // 时间模式按 duration 积分横向 jerk 平方。
    *c_d = poly_d_.J(duration_, 3);
  } else {
    // 弧长模式按实际 delta_s 积分，即使构造时可能使用过 100 m 虚拟跨度。
    *c_d = poly_d_.J(fs1_.vec_s[0] - fs0_.vec_s[0], 3);
  }
  return kSuccess;
}

// 返回横向查询变量在当前模式下的终端跨度。
decimal_t FrenetPrimitive::lateral_T() const {
  if (is_lateral_independent_) {
    return duration_;
  } else {
    return fs1_.vec_s[0] - fs0_.vec_s[0];
  }
}

// 纵向查询变量始终是局部时间。
decimal_t FrenetPrimitive::longitudial_T() const { return duration_; }

// 返回缓存的构造终点状态。
FrenetState FrenetPrimitive::fs1() const { return fs1_; }

// 返回缓存的构造起点状态。
FrenetState FrenetPrimitive::fs0() const { return fs0_; }

}  // namespace common
