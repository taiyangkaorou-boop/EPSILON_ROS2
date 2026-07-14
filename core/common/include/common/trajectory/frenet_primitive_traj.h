#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_PRIMITIVE_TRAJ_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_PRIMITIVE_TRAJ_H__

#include "common/basics/config.h"
#include "common/primitive/frenet_primitive.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"
namespace common {

/**
 * @brief 将 FrenetPrimitive 适配为统一 FrenetTrajectory 接口的轨迹包装器。
 *
 * 包装器按值持有运动基元和 StateTransformer，并提供 12 维多项式系数序列化接口，
 * 便于外部优化器直接调整纵向/横向五次多项式。
 */
class FrenetPrimitiveTrajectory : public FrenetTrajectory {
 public:
  /// 构造无有效基元的轨迹。
  FrenetPrimitiveTrajectory() {}
  /// 复制基元和状态转换器，并将包装器标记为有效。
  FrenetPrimitiveTrajectory(const FrenetPrimitive& primitive,
                            const StateTransformer& stf)
      : primitive_(primitive), stf_(stf), is_valid_(true) {}

  /// 返回基元全局时间起点。
  decimal_t begin() const override { return primitive_.begin(); }
  /// 返回基元全局时间终点。
  decimal_t end() const override { return primitive_.end(); }
  /// 返回包装器构造时维护的有效标记。
  bool IsValid() const override { return is_valid_; }

  /// 查询基元 FrenetState 并转换为世界车辆状态。
  ErrorType GetState(const decimal_t& t, State* state) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    FrenetState fs;
    if (primitive_.GetFrenetState(t, &fs) != kSuccess) {
      return kWrongStatus;
    }

    if (stf_.GetStateFromFrenetState(fs, state) != kSuccess) {
      return kWrongStatus;
    }
    // 当前规划链不保留倒车速度符号，世界速度被截断为非负。
    state->velocity = std::max(0.0, state->velocity);
    return kSuccess;
  }

  /// 在包装器参数域内透传 FrenetPrimitive 的状态查询和错误码语义。
  ErrorType GetFrenetState(const decimal_t& t, FrenetState* fs) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    if (primitive_.GetFrenetState(t, fs) != kSuccess) {
      return kWrongStatus;
    }
    return kSuccess;
  }

  /**
   * @brief 按 `[poly_s 六项, poly_d 六项]` 返回 12 个阶乘缩放系数。
   * @note 不包含基元起始时间、时长、端状态或横向模式标记。
   */
  std::vector<decimal_t> variables() const override {
    std::vector<decimal_t> variables(12);
    Vecf<6> coeff_s = primitive_.poly_s().coeff();
    Vecf<6> coeff_d = primitive_.poly_d().coeff();
    for (int i = 0; i < 6; i++) {
      variables[i] = coeff_s[i];
      variables[i + 6] = coeff_d[i];
    }
    return variables;
  }

  /// 从 12 维向量重建纵向和横向多项式，不改变基元时域及有效标记。
  void set_variables(const std::vector<decimal_t>& variables) override {
    // release 构建仅依赖该 assert 保护固定长度下标访问。
    assert(variables.size() == 12);
    Vecf<6> coeff_s, coeff_d;
    for (int i = 0; i < 6; i++) {
      coeff_s[i] = variables[i];
      coeff_d[i] = variables[6 + i];
    }
    Polynomial<5> poly_s, poly_d;
    poly_s.set_coeff(coeff_s);
    poly_d.set_coeff(coeff_d);
    primitive_.set_poly_s(poly_s);
    primitive_.set_poly_d(poly_d);
  }

  /// 透传基元纵向/横向导数三阶平方积分代价。
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
    primitive_.GetJ(j_lon, j_lat);
  }

 private:
  /// 实际 Frenet 多项式运动基元。
  FrenetPrimitive primitive_;
  /// 参考车道坐标转换器。
  StateTransformer stf_;
  /// 仅由非默认构造函数置真。
  bool is_valid_ = false;
};

}  // namespace common

#endif
