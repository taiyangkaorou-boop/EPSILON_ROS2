#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_

#include <memory>
#include <string>

#include "behavior_planner/map_interface.h"
#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "route_planner/route_planner.h"

#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
namespace planning {

/**
 * @brief 基于多策略决策（MPDM）的车道级行为规划器。
 *
 * 规划器维护自车 Lane 归属与可达左右候选，针对 LK/LCL/LCR 做多车前向 rollout，
 * 评估安全、效率和动作代价，输出 SemanticBehavior 及参考 Lane。
 */
class BehaviorPlanner : public Planner {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  /// 返回规划器显示名称。
  std::string Name() override;

  /// 初始化 RoutePlanner 和行为输出；当前忽略 config 字符串。
  ErrorType Init(const std::string config) override;

  /// 执行车道归属更新、候选决策、MPDM 和参考 Lane 构造。
  ErrorType RunOnce() override;

  /// 注入非拥有的地图接口裸指针。
  void set_map_interface(BehaviorPlannerMapItf* itf);
  /**
   * @brief set desired velocity
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  /**
   * @brief L2-level human-commanded lane changes
   **/
  void set_hmi_behavior(const LateralBehavior& hmi_behavior);

  /**
   * @brief set the level of autonomous driving
   */
  void set_autonomous_level(int level);

  void set_sim_resolution(const decimal_t sim_resolution);

  void set_sim_horizon(const decimal_t sim_horizon);

  void set_use_sim_state(bool use_sim_state);

  void set_aggressive_level(int level);

  /// 更新 RoutePlanner 的地图、自车状态和最近 Lane，并运行一次导航扩展。
  ErrorType RunRoutePlanner(const int nearest_lane_id);

  /// 运行多行为评估并把 winner 写入 behavior_。
  ErrorType RunMpdm();

  Behavior behavior() const;

  decimal_t user_desired_velocity() const;

  decimal_t reference_desired_velocity() const;

  int autonomous_level() const;

  vec_E<vec_E<common::Vehicle>> forward_trajs() const;

  std::vector<LateralBehavior> forward_behaviors() const;

 protected:
  ErrorType ConstructReferenceLane(const LateralBehavior& lat_behavior,
                                   Lane* lane);

  ErrorType ConstructLaneFromSamples(const vec_E<Vecf<2>>& samples, Lane* lane);

  /// 生成可用 LK/LCL/LCR rollout，评估 winner 并输出速度命令。
  ErrorType MultiBehaviorJudge(const decimal_t previous_desired_vel,
                               LateralBehavior* mpdm_behavior,
                               decimal_t* actual_desired_velocity);

  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids);
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  ErrorType MultiAgentSimForward(
      const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  ErrorType OpenloopSimForward(
      const common::SemanticVehicle& ego_semantic_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  ErrorType SimulateEgoBehavior(
      const common::Vehicle& ego_vehicle, const LateralBehavior& ego_behavior,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  ErrorType EvaluateMultiPolicyTrajs(
      const std::vector<LateralBehavior>& valid_behaviors,
      const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
      const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>&
          valid_surround_trajs,
      LateralBehavior* winner_behavior,
      vec_E<common::Vehicle>* winner_forward_traj, decimal_t* winner_score,
      decimal_t* desired_vel);

  ErrorType EvaluateSinglePolicyTraj(
      const LateralBehavior& behaivor,
      const vec_E<common::Vehicle>& forward_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
      decimal_t* score, decimal_t* desired_vel);

  ErrorType EvaluateSafetyCost(const vec_E<common::Vehicle>& traj_a,
                               const vec_E<common::Vehicle>& traj_b,
                               decimal_t* cost);

  ErrorType GetDesiredVelocityOfTrajectory(
      const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel);

  BehaviorPlannerMapItf* map_itf_{nullptr};
  Behavior behavior_;

  planning::RoutePlanner* p_route_planner_{nullptr};

  decimal_t user_desired_velocity_{5.0};
  decimal_t reference_desired_velocity_{5.0};
  int autonomous_level_{3};

  decimal_t sim_resolution_{0.4};
  decimal_t sim_horizon_{4.0};
  int aggressive_level_{3};
  planning::OnLaneForwardSimulation::Param sim_param_;

  bool use_sim_state_ = true;
  bool lock_to_hmi_ = false;
  LateralBehavior hmi_behavior_ = LateralBehavior::kLaneKeeping;

  // track the ego lane id
  int ego_lane_id_{kInvalidLaneId};
  int ego_id_;
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  // debug
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
};

}  // namespace planning

#endif
