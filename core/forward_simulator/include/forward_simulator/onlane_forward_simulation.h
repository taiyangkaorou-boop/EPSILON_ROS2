#ifndef _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_
#define _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_

#include <algorithm>

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "vehicle_model/controllers/ctx_idm_velocity_controller.h"
#include "vehicle_model/controllers/idm_velocity_controller.h"
#include "vehicle_model/controllers/pure_pursuit_controller.h"
#include "vehicle_model/ideal_steer_model.h"

namespace planning {

/**
 * @brief 将车道几何、纵向跟驰控制和理想转向车辆模型组合为单步前向仿真器。
 *
 * 横向使用 Pure Pursuit 计算期望转角，纵向使用 IDM 或 Context-IDM 计算下一步期望
 * 速度，最后交给 IdealSteerModel 在加速度、jerk、曲率和转角约束下积分。所有入口均
 * 为静态函数，不保存跨步状态。
 */
class OnLaneForwardSimulation {
 public:
  /// common/vehicle_model 类型别名。
  using Lane = common::Lane;
  using State = common::State;
  using FrenetState = common::FrenetState;
  using VehicleControlSignal = common::VehicleControlSignal;
  using Vehicle = common::Vehicle;
  using CtxParam = simulator::ContextIntelligentDriverModel::CtxParam;

  /**
   * @brief 单步横纵向控制、车辆动力学限制和失败回退参数。
   */
  struct Param {
    /// IDM/ACC 纵向跟驰参数。
    simulator::IntelligentDriverModel::Param idm_param;
    /// 速度到 Pure Pursuit 前视距离的比例增益。
    decimal_t steer_control_gain = 1.5;
    /// 前视距离上限。
    decimal_t steer_control_max_lookahead_dist = 50.0;
    /// 前视距离下限。
    decimal_t steer_control_min_lookahead_dist = 3.0;
    /// 车辆模型允许的横向加速度绝对值上限。
    decimal_t max_lat_acceleration_abs = 1.5;
    /// 横向 jerk 绝对值上限。
    decimal_t max_lat_jerk_abs = 3.0;
    /// 曲率绝对值上限。
    decimal_t max_curvature_abs = 0.33;
    /// 纵向加速 jerk 上限。
    decimal_t max_lon_acc_jerk = 5.0;
    /// 纵向制动 jerk 上限。
    decimal_t max_lon_brake_jerk = 5.0;
    /// 方向盘/前轮转角绝对值上限。
    decimal_t max_steer_angle_abs = 45.0 / 180.0 * kPi;
    /// 转角变化率上限。
    decimal_t max_steer_rate = 0.39;
    /// 横向投影/转向失败时是否把纵向期望速度降为零。
    bool auto_decelerate_if_lat_failed = true;
  };

  /**
   * @brief 在目标车道前后间隙内生成一个期望纵向位置/速度世界状态。
   *
   * 前车阈值从其后保险杠向后扣除最小间距和自车时距，后车阈值从其前保险杠向前
   * 加入相同规则；随后用硬编码位置误差增益调节参考速度。
   */
  static ErrorType GetTargetStateOnTargetLane(
      const common::StateTransformer& stf_target,
      const common::Vehicle& ego_vehicle,
      const common::Vehicle& gap_front_vehicle,
      const common::Vehicle& gap_rear_vehicle, const Param& param,
      common::State* target_state) {
    common::FrenetState ego_fs;
    // 自车必须能投影到目标车道，否则无法定义间隙纵向坐标。
    if (kSuccess !=
        stf_target.GetFrenetStateFromState(ego_vehicle.state(), &ego_fs)) {
      return kWrongStatus;
    }

    decimal_t time_headaway = param.idm_param.kDesiredHeadwayTime;
    decimal_t min_spacing = param.idm_param.kMinimumSpacing;

    bool has_front = false;
    common::FrenetState front_fs;
    // 前车参考点取后保险杠在目标 Lane 上的近似 s。
    decimal_t s_ref_front = -1;
    decimal_t s_thres_front = -1;
    if (gap_front_vehicle.id() != -1 &&
        kSuccess == stf_target.GetFrenetStateFromState(
                        gap_front_vehicle.state(), &front_fs)) {
      has_front = true;
      s_ref_front =
          front_fs.vec_s[0] - (gap_front_vehicle.param().length() / 2.0 -
                               gap_front_vehicle.param().d_cr());
      s_thres_front = s_ref_front - min_spacing -
                      time_headaway * ego_vehicle.state().velocity;
    }

    bool has_rear = false;
    common::FrenetState rear_fs;
    // 后车参考点取前保险杠在目标 Lane 上的近似 s。
    decimal_t s_ref_rear = -1;
    decimal_t s_thres_rear = -1;
    if (gap_rear_vehicle.id() != -1 &&
        kSuccess == stf_target.GetFrenetStateFromState(gap_rear_vehicle.state(),
                                                       &rear_fs)) {
      has_rear = true;
      s_ref_rear = rear_fs.vec_s[0] + gap_rear_vehicle.param().length() / 2.0 +
                   gap_rear_vehicle.param().d_cr();
      s_thres_rear = s_ref_rear + min_spacing +
                     time_headaway * gap_rear_vehicle.state().velocity;
    }

    decimal_t desired_s = ego_fs.vec_s[0];
    decimal_t desired_v = ego_vehicle.state().velocity;

    // 间隙速度调节参数硬编码：位置误差乘 0.1，速度修正截断到 [-3,5] m/s。
    decimal_t k_v = 0.1;
    decimal_t p_v_ego = 0.1;
    decimal_t dv_lb = -3.0;
    decimal_t dv_ub = 5.0;

    // 自车偏好速度每次只向 IDM 期望速度移动 10%。
    decimal_t ego_desired_vel =
        ego_vehicle.state().velocity +
        (param.idm_param.kDesiredVelocity - ego_vehicle.state().velocity) *
            p_v_ego;
    if (has_front && has_rear) {
      // 同时存在前后车时，保险杠参考点顺序必须形成非负间隙。
      if (s_ref_front < s_ref_rear) {
        return kWrongStatus;
      }

      decimal_t ds = fabs(s_ref_front - s_ref_rear);
      // s_star 以间隙中点为几何目标，并修正自车后轴到几何中心偏置。
      decimal_t s_star = s_ref_rear + ds / 2.0 - ego_vehicle.param().d_cr();

      s_thres_front = std::max(s_star, s_thres_front);
      s_thres_rear = std::min(s_star, s_thres_rear);

      desired_s =
          std::min(std::max(s_thres_rear, ego_fs.vec_s[0]), s_thres_front);

      decimal_t s_err_front = s_thres_front - ego_fs.vec_s[0];
      decimal_t v_ref_front =
          std::max(0.0, gap_front_vehicle.state().velocity +
                            truncate(s_err_front * k_v, dv_lb, dv_ub));

      decimal_t s_err_rear = s_thres_rear - ego_fs.vec_s[0];
      decimal_t v_ref_rear =
          std::max(0.0, gap_rear_vehicle.state().velocity +
                            truncate(s_err_rear * k_v, dv_lb, dv_ub));

      desired_v = std::min(std::max(v_ref_rear, ego_desired_vel), v_ref_front);

    } else if (has_front) {
      // 只有前车时，期望位置/速度均不越过前方安全阈值。
      desired_s = std::min(ego_fs.vec_s[0], s_thres_front);

      decimal_t s_err_front = s_thres_front - ego_fs.vec_s[0];
      decimal_t v_ref_front =
          std::max(0.0, gap_front_vehicle.state().velocity +
                            truncate(s_err_front * k_v, dv_lb, dv_ub));
      desired_v = std::min(ego_desired_vel, v_ref_front);

    } else if (has_rear) {
      // 只有后车时，期望位置/速度至少满足后方车辆所需阈值。
      desired_s = std::max(ego_fs.vec_s[0], s_thres_rear);

      decimal_t s_err_rear = s_thres_rear - ego_fs.vec_s[0];
      decimal_t v_ref_rear =
          std::max(0.0, gap_rear_vehicle.state().velocity +
                            truncate(s_err_rear * k_v, dv_lb, dv_ub));
      desired_v = std::max(v_ref_rear, ego_desired_vel);
    }

    common::FrenetState target_fs;
    // 目标横向状态固定在 Lane 中心，纵向加速度置零；时间戳不从自车复制。
    target_fs.Load(Vecf<3>(desired_s, desired_v, 0.0), Vecf<3>(0.0, 0.0, 0.0),
                   common::FrenetState::kInitWithDs);

    if (kSuccess !=
        stf_target.GetStateFromFrenetState(target_fs, target_state)) {
      return kWrongStatus;
    }

    return kSuccess;
  }

  /**
   * @brief 带横向跟踪偏移的高级保持车道单步传播。
   *
   * 在同一 Lane 上计算带 offset 的 Pure Pursuit 转角，再按有无前车选择真实/虚拟前车
   * IDM，最后由 IdealSteerModel 积分。
   */
  static ErrorType PropagateOnceAdvancedLK(
      const common::StateTransformer& stf, const common::Vehicle& ego_vehicle,
      const Vehicle& leading_vehicle, const decimal_t& lat_track_offset,
      const decimal_t& dt, const Param& param, State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    // 阶段一：投影自车并计算横向期望转角。
    bool steer_calculation_failed = false;
    common::FrenetState current_fs;
    if (stf.GetFrenetStateFromState(current_state, &current_fs) != kSuccess ||
        current_fs.vec_s[1] < -kEPS) {
      // 投影失败或 Frenet 纵向速度为负时保留当前转角。
      steer_calculation_failed = true;
    }

    decimal_t steer, velocity;
    if (!steer_calculation_failed) {
      // 前视距离按车速线性增长，并截断到配置上下限。
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (CalcualateSteer(stf, current_state, current_fs, wheelbase_len,
                          Vec2f(approx_lookahead_dist, lat_track_offset),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      // 横向失败可触发期望速度归零，但后续仍继续执行纵向和车辆模型。
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // 阶段二：无前车使用远端虚拟前车，有前车则计算等效车长后的 IDM 速度。
    common::FrenetState leading_fs;
    if (leading_vehicle.id() == kInvalidAgentId ||
        stf.GetFrenetStateFromState(leading_vehicle.state(), &leading_fs) !=
            kSuccess) {
      CalcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                 &velocity);
    } else {
      // 以两车保险杠几何修正 IDM 固定车长，使后轴 s 差对应净间距。
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf, ego_vehicle, leading_vehicle,
                                    leading_fs, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingIdm(
          current_fs.vec_s[0], current_state.velocity, leading_fs.vec_s[0],
          leading_vehicle.state().velocity, dt, sim_param, &velocity);
    }

    // 阶段三：理想转向车辆模型施加动态限制并积分一个 dt。
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  /**
   * @brief 面向换道的高级单步传播：横向追踪目标 Lane，纵向融合当前 Lane 前车与目标间隙。
   */
  static ErrorType PropagateOnceAdvancedLC(
      const common::StateTransformer& stf_current,
      const common::StateTransformer& stf_target,
      const common::Vehicle& ego_vehicle,
      const Vehicle& current_leading_vehicle, const Vehicle& gap_front_vehicle,
      const Vehicle& gap_rear_vehicle, const decimal_t& lat_track_offset,
      const decimal_t& dt, const Param& param, State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    decimal_t steer, velocity;

    // 阶段一：把自车投影到目标 Lane 并计算带 offset 的 Pure Pursuit 转角。
    bool steer_calculation_failed = false;
    common::FrenetState ego_on_tarlane_fs;
    if (stf_target.GetFrenetStateFromState(current_state, &ego_on_tarlane_fs) !=
            kSuccess ||
        ego_on_tarlane_fs.vec_s[1] < -kEPS) {
      // 目标 Lane 投影失败或逆行时保留当前转角。
      steer_calculation_failed = true;
    }

    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (CalcualateSteer(stf_target, current_state, ego_on_tarlane_fs,
                          wheelbase_len,
                          Vec2f(approx_lookahead_dist, lat_track_offset),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // 阶段二：先在目标 Lane 间隙内生成期望状态，失败则退回当前世界状态。
    common::State target_state;
    if (kSuccess != GetTargetStateOnTargetLane(
                        stf_target, ego_vehicle, gap_front_vehicle,
                        gap_rear_vehicle, sim_param, &target_state)) {
      target_state = current_state;
    }
    common::FrenetState target_on_curlane_fs;
    // 下面两个投影失败分支为空，失败输出随后仍会被 Context-IDM 使用。
    if (stf_current.GetFrenetStateFromState(
            target_state, &target_on_curlane_fs) != kSuccess) {
    }

    common::FrenetState ego_on_curlane_fs;
    if (stf_current.GetFrenetStateFromState(current_state,
                                            &ego_on_curlane_fs) != kSuccess) {
    }

    // Context-IDM 的上下文融合权重固定为 (0.4,0.8)，不属于 Param 可配置项。
    simulator::ContextIntelligentDriverModel::CtxParam ctx_param(0.4, 0.8);

    common::FrenetState current_leading_fs;
    if (current_leading_vehicle.id() == kInvalidAgentId ||
        stf_current.GetFrenetStateFromState(current_leading_vehicle.state(),
                                            &current_leading_fs) != kSuccess) {
      // 无当前 Lane 前车时使用虚拟前车，只融合目标间隙状态。
      CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    } else {
      // 有当前 Lane 前车时加入其约束，并用保险杠几何修正等效车长。
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf_current, ego_vehicle,
                                    current_leading_vehicle, current_leading_fs,
                                    &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          current_leading_fs.vec_s[0], current_leading_vehicle.state().velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    }

    // 阶段三：使用同一 IdealSteerModel 积分输出状态。
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  /**
   * @brief 标准保持车道单步传播，等价于横向 offset=0 的 LK 控制流程。
   */
  static ErrorType PropagateOnce(const common::StateTransformer& stf,
                                 const common::Vehicle& ego_vehicle,
                                 const Vehicle& leading_vehicle,
                                 const decimal_t& dt, const Param& param,
                                 State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    // 阶段一：投影自车，按速度自适应前视距离追踪 Lane 中心。
    bool steer_calculation_failed = false;
    common::FrenetState current_fs;
    if (stf.GetFrenetStateFromState(current_state, &current_fs) != kSuccess ||
        current_fs.vec_s[1] < -kEPS) {
      // 投影失败或 Frenet 纵向速度为负时保留当前转角。
      steer_calculation_failed = true;
    }

    decimal_t steer, velocity;
    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (CalcualateSteer(stf, current_state, current_fs, wheelbase_len,
                          Vec2f(approx_lookahead_dist, 0.0),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // 阶段二：按有无可投影前车选择虚拟/真实前车 IDM。
    common::FrenetState leading_fs;
    if (leading_vehicle.id() == kInvalidAgentId ||
        stf.GetFrenetStateFromState(leading_vehicle.state(), &leading_fs) !=
            kSuccess) {
      CalcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                 &velocity);
    } else {
      // 使用等效车长把后轴 s 差修正为保险杠净间距。
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf, ego_vehicle, leading_vehicle,
                                    leading_fs, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingIdm(
          current_fs.vec_s[0], current_state.velocity, leading_fs.vec_s[0],
          leading_vehicle.state().velocity, dt, sim_param, &velocity);
    }

    // 阶段三：在 IdealSteerModel 动力学限制下积分输出状态。
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  /**
   * @brief 无 Lane 时保持当前转角/当前速度，通过 IdealSteerModel 积分一步。
   * @param desired_vel 名义期望速度；当前实现未使用该参数。
   */
  static ErrorType PropagateOnce(const decimal_t& desired_vel,
                                 const common::Vehicle& ego_vehicle,
                                 const decimal_t& dt, const Param& param,
                                 State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    // baseline 忽略 desired_vel，控制速度直接取当前速度。
    decimal_t steer = current_state.steer;
    decimal_t velocity = current_state.velocity;
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          param, desired_state);
    return kSuccess;
  }

 private:
  /**
   * @brief 根据前车保险杠在参考 Lane 上的最近 s，计算 IDM 等效车辆长度。
   */
  static ErrorType GetIdmEquivalentVehicleLength(
      const common::StateTransformer& stf, const common::Vehicle& ego_vehicle,
      const common::Vehicle& leading_vehicle,
      const common::FrenetState& leading_fs, decimal_t* eqv_vehicle_len) {
    // 状态位置保留后轴中心；把自车前悬和前车后保险杠到后轴距离合并进 IDM 车长。
    // 同时投影前车两个保险杠，缓解感知航向翻转时前后端交换问题。
    std::array<Vec2f, 2> leading_pts;
    leading_vehicle.RetBumperVertices(&leading_pts);

    Vec2f fs_pt1, fs_pt2;
    std::vector<decimal_t> s_vec;
    if (kSuccess == stf.GetFrenetPointFromPoint(leading_pts[0], &fs_pt1)) {
      s_vec.push_back(fs_pt1(0));
    }
    if (kSuccess == stf.GetFrenetPointFromPoint(leading_pts[1], &fs_pt2)) {
      s_vec.push_back(fs_pt2(0));
    }
    s_vec.push_back(leading_fs.vec_s(0));
    // 即使两个保险杠投影都失败，前车后轴 s 仍保证容器非空。
    decimal_t s_nearest_vtx = *(std::min_element(s_vec.begin(), s_vec.end()));

    decimal_t len_rb2r = fabs(leading_fs.vec_s(0) - s_nearest_vtx);
    // 等效长度=自车后轴到前保险杠+前车最近后端到其后轴。
    *eqv_vehicle_len = ego_vehicle.param().length() / 2.0 +
                       ego_vehicle.param().d_cr() + len_rb2r;

    return kSuccess;
  }

  /**
   * @brief 将当前 Frenet s 加前视距离、目标 d 设为 offset，转换世界点后执行 Pure Pursuit。
   */
  static ErrorType CalcualateSteer(const common::StateTransformer& stf,
                                   const State& current_state,
                                   const FrenetState& current_fs,
                                   const decimal_t& wheelbase_len,
                                   const Vec2f& lookahead_offset,
                                   decimal_t* steer) {
    common::FrenetState dest_fs;
    dest_fs.Load(Vecf<3>(lookahead_offset(0) + current_fs.vec_s[0], 0.0, 0.0),
                 Vecf<3>(lookahead_offset(1), 0.0, 0.0),
                 common::FrenetState::kInitWithDs);

    State dest_state;
    if (stf.GetStateFromFrenetState(dest_fs, &dest_state) != kSuccess) {
      return kWrongStatus;
    }

    // 使用真实世界直线距离和朝向误差，而不是直接使用 Frenet 前视 s。
    decimal_t look_ahead_dist =
        (dest_state.vec_position - current_state.vec_position).norm();
    decimal_t cur_to_dest_angle =
        vec2d_to_angle(dest_state.vec_position - current_state.vec_position);
    decimal_t angle_diff =
        normalize_angle(cur_to_dest_angle - current_state.angle);
    control::PurePursuitControl::CalculateDesiredSteer(
        wheelbase_len, angle_diff, look_ahead_dist, steer);
    return kSuccess;
  }

  /// 使用真实前车位置/速度，通过 IDM 速度控制器积分得到下一步期望速度。
  static ErrorType CalcualateVelocityUsingIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& leading_pos, const decimal_t& leading_vel,
      const decimal_t& dt, const Param& param, decimal_t* velocity) {
    decimal_t leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      // 前车倒车速度在纵向跟驰模型中按静止处理。
      leading_vel_fin = 0;
    }
    // current_vel/leading_vel 使用车身标量速度；高曲率下 Frenet s_dot 可能大于车身速度。
    return control::IntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, current_pos, leading_pos, current_vel, leading_vel_fin,
        dt, velocity);
  }

  /// 无真实前车时，在前方 `100+100*current_vel` 处构造同速虚拟前车。
  static ErrorType CalcualateVelocityUsingIdm(const decimal_t& current_vel,
                                              const decimal_t& dt,
                                              const Param& param,
                                              decimal_t* velocity) {
    const decimal_t virtual_leading_dist = 100.0 + 100.0 * current_vel;
    return control::IntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, 0.0, 0.0 + virtual_leading_dist, current_vel,
        current_vel, dt, velocity);
  }

  /// 使用真实当前 Lane 前车和目标间隙状态执行 Context-IDM 速度控制。
  static ErrorType CalcualateVelocityUsingCtxIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& leading_pos, const decimal_t& leading_vel,
      const decimal_t& target_pos, const decimal_t& target_vel,
      const decimal_t& dt, const Param& param, const CtxParam& ctx_param,
      decimal_t* velocity) {
    decimal_t leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      // 与普通 IDM 一致，负前车速度按零处理。
      leading_vel_fin = 0;
    }
    // 输入速度使用车辆标量速度，避免高曲率 Frenet s_dot 的尺度放大。
    return control::ContextIntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, ctx_param, current_pos, leading_pos, target_pos,
        current_vel, leading_vel_fin, target_vel, dt, velocity);
  }

  /// 无真实当前 Lane 前车时，以远端同速虚拟前车执行 Context-IDM。
  static ErrorType CalcualateVelocityUsingCtxIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& target_pos, const decimal_t& target_vel,
      const decimal_t& dt, const Param& param, const CtxParam& ctx_param,
      decimal_t* velocity) {
    const decimal_t virtual_leading_pos =
        current_pos + 100.0 + 100.0 * current_vel;
    return control::ContextIntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, ctx_param, current_pos, virtual_leading_pos,
        target_pos, current_vel, current_vel, target_vel, dt, velocity);
  }

  /**
   * @brief 构造 IdealSteerModel，施加所有动态限制并积分 dt，更新时间戳。
   */
  static ErrorType CalculateDesiredState(const State& current_state,
                                         const decimal_t steer,
                                         const decimal_t velocity,
                                         const decimal_t wheelbase_len,
                                         const decimal_t dt, const Param& param,
                                         State* state) {
    // 每个单步调用都新建模型，不跨周期保留积分器内部状态。
    simulator::IdealSteerModel model(
        wheelbase_len, param.idm_param.kAcceleration,
        param.idm_param.kHardBrakingDeceleration, param.max_lon_acc_jerk,
        param.max_lon_brake_jerk, param.max_lat_acceleration_abs,
        param.max_lat_jerk_abs, param.max_steer_angle_abs, param.max_steer_rate,
        param.max_curvature_abs);
    model.set_state(current_state);
    model.set_control(simulator::IdealSteerModel::Control(steer, velocity));
    model.Step(dt);
    *state = model.state();
    state->time_stamp = current_state.time_stamp + dt;
    return kSuccess;
  }
};

}  // namespace planning

#endif  // _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_
