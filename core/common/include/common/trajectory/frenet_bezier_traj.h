#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__

#include "common/basics/config.h"
#include "common/spline/bezier.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"

namespace common {

/**
 * @brief 以二维时间缩放 Bezier 样条表示 `[s(t), d(t)]` 的 Frenet 轨迹。
 *
 * 对象按值持有 Bezier 样条和参考车道转换器。默认采用横向时间独立模式，查询时先
 * 生成 FrenetState，再转换为世界 State。
 */
class FrenetBezierTrajectory : public FrenetTrajectory {
 public:
  /// 项目固定使用五次二维 Bezier 样条表达纵向 s 与横向 d。
  using BezierTrajectory = BezierSpline<TrajectoryDegree, TrajectoryDim>;

  /// 构造无有效样条的轨迹。
  FrenetBezierTrajectory() {}
  /// 复制样条与状态转换器，并将轨迹标记为有效。
  FrenetBezierTrajectory(const BezierTrajectory& bezier_spline,
                         const StateTransformer& stf)
      : bezier_spline_(bezier_spline), stf_(stf), is_valid_(true) {}

  /// 返回底层 Bezier 样条参数域起点。
  decimal_t begin() const override { return bezier_spline_.begin(); }
  /// 返回底层 Bezier 样条参数域终点。
  decimal_t end() const override { return bezier_spline_.end(); }
  /// 返回构造函数维护的有效标记，不复查样条断点或控制变量。
  bool IsValid() const override { return is_valid_; }

  /// 查询 Frenet 状态并通过参考车道转换为世界车辆状态。
  ErrorType GetState(const decimal_t& t, State* state) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    common::FrenetState fs;
    if (GetFrenetState(t, &fs) != kSuccess) {
      return kWrongStatus;
    }
    if (stf_.GetStateFromFrenetState(fs, state) != kSuccess) {
      return kWrongStatus;
    }
    // 当前规划链不允许输出负速度，世界转换结果在此被截断为非负。
    state->velocity = std::max(0.0, state->velocity);
    return kSuccess;
  }

  /// 从 Bezier 位置、一阶导和二阶导构造 t 参数化的 FrenetState。
  ErrorType GetFrenetState(const decimal_t& t, FrenetState* fs) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    Vecf<2> pos, vel, acc;
    bezier_spline_.evaluate(t, 0, &pos);
    bezier_spline_.evaluate(t, 1, &vel);
    bezier_spline_.evaluate(t, 2, &acc);
    // 默认把纵向和横向均解释为对时间的导数，即 lateral-independent 模式。
    fs->time_stamp = t;
    fs->Load(Vec3f(pos[0], vel[0], acc[0]), Vec3f(pos[1], vel[1], acc[1]),
             common::FrenetState::kInitWithDt);
    if (!fs->is_ds_usable) {
      // 纵向速度过小时无法稳定换算 d 对 s 的导数，退化为仅保留 s/d 位置。
      fs->Load(Vec3f(pos[0], 0.0, 0.0), Vec3f(pos[1], 0.0, 0.0),
               FrenetState::kInitWithDs);
    }
    return kSuccess;
  }

  /// 优化变量序列化尚未实现，当前固定返回空向量。
  std::vector<decimal_t> variables() const override {
    return std::vector<decimal_t>();
  }

  /// 优化变量回写尚未实现，当前忽略全部输入。
  void set_variables(const std::vector<decimal_t>& variables) override {
  }

  /// jerk 指标尚未实现，当前不会写入两个输出指针。
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
  }

 private:
  /// 时间缩放 `[s,d]` Bezier 样条。
  BezierTrajectory bezier_spline_;
  /// Frenet/世界坐标转换器，内部按值持有参考 Lane。
  StateTransformer stf_;
  /// 仅由非默认构造函数置真。
  bool is_valid_ = false;
};

}  // namespace common

#endif
