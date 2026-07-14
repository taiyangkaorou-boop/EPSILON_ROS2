#ifndef _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__
#define _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__

#include "common/spline/polynomial.h"
#include "common/state/frenet_state.h"

namespace common {

/**
 * @brief 由纵向/横向五次多项式组成的单段 Frenet 运动基元。
 *
 * 纵向始终表示 s(t)。横向可选择 d(t) 的时间独立模式，或 d(delta_s) 的弧长模式。
 * 对象保存全局起始时间、持续时长和两端 FrenetState，并允许域外解析外推。
 */
class FrenetPrimitive {
 public:
  /// 构造零时长、全零多项式的无有效运动基元。
  FrenetPrimitive() {}

  /**
   * @brief 用两端 Frenet 状态构造纵横向 jerk-optimal 五次连接。
   * @param fs0 起始 Frenet 状态。
   * @param fs1 终止 Frenet 状态。
   * @param stamp 全局起始时间。
   * @param T 时间持续长度。
   * @param is_lateral_independent true 使用 d(t)，false 使用 d(delta_s)。
   */
  ErrorType Connect(const FrenetState& fs0, const FrenetState& fs1,
                    const decimal_t stamp, const decimal_t T,
                    bool is_lateral_independent);
  /**
   * @brief 以恒定纵/横向加速度控制从起始状态解析传播。
   * @param fs0 起始 Frenet 状态。
   * @param u `[s_ddot,d_ddot]` 控制。
   * @param stamp 全局起始时间。
   * @param T 持续时间。
   */
  ErrorType Propagate(const FrenetState& fs0, const Vecf<2>& u,
                      const decimal_t stamp, const decimal_t T);

  /// 返回全局时间域起点。
  decimal_t begin() const { return stamp_; }

  /// 返回全局时间域终点 `stamp_+duration_`。
  decimal_t end() const { return stamp_ + duration_; }

  /**
   * @brief 查询全局时间 t_global 对应的 Frenet 状态。
   * @note 不拒绝时间域外参数，会使用多项式外推；duration 小于 kEPS 时返回错误。
   */
  ErrorType GetFrenetState(const decimal_t t_global, FrenetState* fs) const;

  /// 从 begin()+offset 起以固定步长采样至 end() 之前，覆盖输出容器。
  ErrorType GetFrenetStateSamples(const decimal_t step, const decimal_t offset,
                                  vec_E<FrenetState>* fs_vec) const;

  /// 输出纵向和横向三阶导数平方积分指标。
  ErrorType GetJ(decimal_t* c_s, decimal_t* c_d) const;

  /// 返回横向多项式使用的实际查询跨度：时间模式为 duration，弧长模式为 delta_s。
  decimal_t lateral_T() const;
  /// 返回纵向多项式时间跨度。
  decimal_t longitudial_T() const;

  /// 返回终止 FrenetState 副本。
  FrenetState fs1() const;
  /// 返回起始 FrenetState 副本。
  FrenetState fs0() const;

  /// 返回纵向五次多项式副本。
  Polynomial<5> poly_s() const { return poly_s_; }
  /// 返回横向五次多项式副本。
  Polynomial<5> poly_d() const { return poly_d_; }

  /// 只替换纵向多项式，不同步端状态、时域或有效性。
  void set_poly_s(const Polynomial<5>& poly) { poly_s_ = poly; }
  /// 只替换横向多项式，不同步端状态、时域或有效性。
  void set_poly_d(const Polynomial<5>& poly) { poly_d_ = poly; }

  /// 打印时域、两条多项式系数和两端 FrenetState。
  void print() const {
    printf("frenet primitive in duration [%lf, %lf].\n", begin(), end());
    poly_s_.print();
    poly_d_.print();
    fs0_.print();
    fs1_.print();
  }

  /// 横向是否以时间独立参数化；该字段公开可写，调用方可改变解释语义。
  bool is_lateral_independent_ = false;

 private:
  /// 纵向 s(t) 五次多项式。
  Polynomial<5> poly_s_;
  /// 横向 d(t) 或 d(delta_s) 五次多项式。
  Polynomial<5> poly_d_;
  /// 全局起始时间。
  decimal_t stamp_{0.0};
  /// 时间持续长度。
  decimal_t duration_{0.0};
  /// 构造时保存的起始状态。
  FrenetState fs0_;
  /// 构造或传播得到的终止状态。
  FrenetState fs1_;
  /// 弧长模式小纵向位移时改用 100 m 虚拟连接跨度的触发阈值。
  decimal_t kSmallDistanceThreshold_ = 2.0;
};

}  // namespace common

#endif
