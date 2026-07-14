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

  /// 在自动驾驶等级不低于 L2 时更新用户期望速度，并把负值截断为零。
  void set_user_desired_velocity(const decimal_t desired_vel);

  /// 接收 HMI 横向行为：L2 立即写入行为，L3 仅锁定 MPDM 的横向 winner。
  void set_hmi_behavior(const LateralBehavior& hmi_behavior);

  /// 直接设置自动驾驶等级；当前不校验取值范围。
  void set_autonomous_level(int level);

  /// 设置 rollout 离散时间步长；当前不校验正值。
  void set_sim_resolution(const decimal_t sim_resolution);

  /// 设置 rollout 预测时域；当前不校验正值。
  void set_sim_horizon(const decimal_t sim_horizon);

  /// 选择使用仿真自车状态还是真实自车状态参与 Lane 归属判断。
  void set_use_sim_state(bool use_sim_state);

  /// 设置 1--5 驾驶激进程度，下一次 L3 规划时映射为前向仿真参数。
  void set_aggressive_level(int level);

  /// 更新 RoutePlanner 的地图、自车状态和最近 Lane，并运行一次导航扩展。
  ErrorType RunRoutePlanner(const int nearest_lane_id);

  /// 运行多行为评估并把 winner 写入 behavior_。
  ErrorType RunMpdm();

  /// 返回当前 SemanticBehavior 的值拷贝。
  Behavior behavior() const;

  /// 返回用户设置并经非负截断的期望速度。
  decimal_t user_desired_velocity() const;

  /// 返回最终参考 Lane 曲率约束后的内部参考速度。
  decimal_t reference_desired_velocity() const;

  /// 返回当前自动驾驶等级。
  int autonomous_level() const;

  /// 返回本周期所有有效自车候选 rollout 的值拷贝。
  vec_E<vec_E<common::Vehicle>> forward_trajs() const;

  /// 返回与 forward_trajs() 按下标对应的横向行为值拷贝。
  std::vector<LateralBehavior> forward_behaviors() const;

 protected:
  /// 按最终横向行为选择目标 Lane，拟合局部参考线并更新曲率约束参考速度。
  /// 相邻 Lane 不可用时回退当前 Lane 并把输出行为改为 LK。
  ErrorType ConstructReferenceLane(const LateralBehavior& lat_behavior,
                                   Lane* lane);

  /// 以样本折线累计弦长为参数，用固定 20 个 break 和正则项拟合 Lane。
  ErrorType ConstructLaneFromSamples(const vec_E<Vecf<2>>& samples, Lane* lane);

  /// 生成可用 LK/LCL/LCR rollout，评估 winner 并输出速度命令。
  ErrorType MultiBehaviorJudge(const decimal_t previous_desired_vel,
                               LateralBehavior* mpdm_behavior,
                               decimal_t* actual_desired_velocity);

  /// 获取源 Lane 在 LK/LCL/LCR 下可观测到的直接目标 Lane 及其子 Lane ID。
  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids);

  /// 更新当前自车 Lane ID，并重建 LK/LCL/LCR 三组潜在 Lane 缓存。
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  /// 根据旧 Lane 及潜在 Lane 缓存解释新观测 Lane 对应的横向行为。
  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  /// 用观测横向行为推进 LK/LCL/LCR 状态机，并在换道结束或异常时解除 HMI 锁定。
  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  /// 在固定参考车道上同步滚动自车与周车，并记录各车完整预测轨迹。
  ///
  /// semantic_vehicle_set 必须包含 ego_id 对应的语义车辆。每个仿真步先基于
  /// 同一时刻的车辆集合计算全部下一状态，再统一提交，避免车辆遍历顺序污染结果。
  /// traj 输出自车轨迹，surround_trajs 按车辆 ID 输出周车轨迹，二者都包含初始状态。
  /// 当前状态已碰撞、前向传播失败等情况返回 kWrongStatus，由上层决定是否降级。
  ErrorType MultiAgentSimForward(
      const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  /// 多车交互仿真失败时使用的独立开环降级预测。
  ///
  /// 自车和每辆周车均沿各自固定参考车道传播，不查询前车，也不建模车辆间响应；
  /// 自车使用规划参考速度，周车保持各自初始速度作为期望速度。输出轨迹包含初始
  /// 状态；任一车辆传播失败或自车预测状态被地图判定碰撞时返回 kWrongStatus。
  ErrorType OpenloopSimForward(
      const common::SemanticVehicle& ego_semantic_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  /// 为一个候选横向行为构造自车参考车道并生成联合预测轨迹。
  ///
  /// 函数先把自车加入语义车辆集合执行多车交互 rollout；若交互仿真失败，则自动
  /// 回退到独立开环预测。traj 和 surround_trajs 分别返回自车与周车的时序状态，
  /// 参考车道构造失败或两级预测均失败时返回 kWrongStatus。
  ErrorType SimulateEgoBehavior(
      const common::Vehicle& ego_vehicle, const LateralBehavior& ego_behavior,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  /// 逐一评估有效候选轨迹，并以总代价最小者作为 MPDM winner。
  ///
  /// valid_behaviors、valid_forward_trajs 和 valid_surround_trajs 需要按下标一一对应。
  /// 输出 winner 的横向行为、自车轨迹、总代价和建议速度；候选集合为空时返回
  /// kWrongStatus，相同代价时保留候选序列中更靠前的行为。
  ErrorType EvaluateMultiPolicyTrajs(
      const std::vector<LateralBehavior>& valid_behaviors,
      const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
      const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>&
          valid_surround_trajs,
      LateralBehavior* winner_behavior,
      vec_E<common::Vehicle>* winner_forward_traj, decimal_t* winner_score,
      decimal_t* desired_vel);

  /// 计算单个候选行为的效率、安全和换道动作代价，并给出轨迹建议速度。
  ///
  /// forward_traj 为自车 rollout，surround_traj 按车辆 ID 保存对应周车 rollout。
  /// 当前代价只使用终端速度/前车、逐时刻车辆碰撞和固定换道惩罚，三项直接相加。
  ErrorType EvaluateSinglePolicyTraj(
      const LateralBehavior& behaivor,
      const vec_E<common::Vehicle>& forward_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
      decimal_t* score, decimal_t* desired_vel);

  /// 对齐比较两条等长车辆轨迹，累计膨胀车身相交时的相对速度软惩罚。
  ///
  /// 每个采样点将两车宽度和长度各增加 1 m；发生碰撞时累加
  /// 0.005*|速度差|。轨迹长度不同返回 kWrongStatus，空的等长轨迹代价为零。
  ErrorType EvaluateSafetyCost(const vec_E<common::Vehicle>& traj_a,
                               const vec_E<common::Vehicle>& traj_b,
                               decimal_t* cost);

  /// 按 baseline 的横向加速度扫描规则从自车轨迹提取建议速度。
  ///
  /// 当前实现会在每个非零 |curvature|*velocity^2 采样处覆盖候选速度，因此实际
  /// 返回最后一个满足条件的状态速度；若全部横向加速度为零，则输出保持为 kInf。
  ErrorType GetDesiredVelocityOfTrajectory(
      const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel);

  // 外部注入且不由本类释放的地图接口。
  BehaviorPlannerMapItf* map_itf_{nullptr};
  Behavior behavior_;

  // Init 中动态创建并由当前类长期持有的导航 RoutePlanner。
  planning::RoutePlanner* p_route_planner_{nullptr};

  // 用户速度上限与最终参考 Lane 曲率约束后的内部参考速度。
  decimal_t user_desired_velocity_{5.0};
  decimal_t reference_desired_velocity_{5.0};
  int autonomous_level_{3};

  // 前向 rollout 的离散参数、驾驶风格等级及对应车辆传播参数。
  decimal_t sim_resolution_{0.4};
  decimal_t sim_horizon_{4.0};
  int aggressive_level_{3};
  planning::OnLaneForwardSimulation::Param sim_param_;

  bool use_sim_state_ = true;
  bool lock_to_hmi_ = false;
  LateralBehavior hmi_behavior_ = LateralBehavior::kLaneKeeping;

  // 当前 Lane 与基于其拓扑预计算的同向、左换道和右换道可观测 Lane 集合。
  int ego_lane_id_{kInvalidLaneId};
  int ego_id_;
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  // 本周期有效候选及周车联合 rollout，既用于行为输出也用于调试可视化。
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
};

}  // namespace planning

#endif
