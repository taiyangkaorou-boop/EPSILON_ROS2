#ifndef _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_
#define _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/**
 * @brief 使用 OnLaneForwardSimulation 对单车做开环短时状态序列预测。
 *
 * 有有效 Lane 时沿车道传播，无有效 Lane 时使用自由空间重载；不输入前车，期望速度
 * 固定为车辆当前速度。该预测器只提供静态函数，不保存模型状态。
 */
class OnLaneFsPredictor {
 public:
  /// common 类型别名，缩短预测接口签名。
  using Lane = common::Lane;
  using State = common::State;
  using VehicleControlSignal = common::VehicleControlSignal;
  using Vehicle = common::Vehicle;

  /// 默认构造不持有资源。
  OnLaneFsPredictor() {}
  /// 析构函数当前只有声明，仓库内没有对应定义。
  ~OnLaneFsPredictor();

  /**
   * @brief 从当前状态开始，以固定步长生成开环预测状态序列。
   * @param lane 参考车道；无效时退化为自由空间传播。
   * @param vehicle 待预测车辆。
   * @param t_pred 期望预测时长。
   * @param t_step 单步时长，baseline 要求为正。
   * @param pred_states 输出容器；先清空并包含初始状态。
   * @note 预测步数使用 round(t_pred/t_step)，实际终点可能早于或晚于 t_pred。
   */
  static ErrorType GetPredictedTrajectory(const Lane& lane,
                                          const Vehicle& vehicle,
                                          const decimal_t& t_pred,
                                          const decimal_t& t_step,
                                          vec_E<State>* pred_states) {
    pred_states->clear();
    // 未验证 t_step 非零/为正，round 结果再窄化为 int。
    int num_step = std::round(t_pred / t_step);
    State desired_state;
    decimal_t desired_vel = vehicle.state().velocity;
    planning::OnLaneForwardSimulation::Param sim_param;
    // 期望速度设为当前速度，使自由道路预测倾向保持当前纵向速度。
    sim_param.idm_param.kDesiredVelocity = desired_vel;
    // 输出第一个元素始终是输入车辆当前状态。
    pred_states->push_back(vehicle.state());
    common::Vehicle v_in = vehicle;
    common::StateTransformer stf = common::StateTransformer(lane);
    for (int i = 0; i < num_step; ++i) {
      if (lane.IsValid()) {
        // 默认 Vehicle 表示没有显式前车，单步失败时保留已生成的输出前缀。
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                stf, v_in, common::Vehicle(), t_step, sim_param,
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      } else {
        // 无车道时使用期望航向/速度的自由空间传播重载。
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                desired_vel, v_in, t_step,
                planning::OnLaneForwardSimulation::Param(),
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      }
      pred_states->push_back(desired_state);
      // 下一步以前一步期望状态作为新的开环输入。
      v_in.set_state(desired_state);
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif  // _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_
