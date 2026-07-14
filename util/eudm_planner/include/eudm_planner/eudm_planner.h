/**
 * @file eudm_planner.h
 * @brief EUDM 离散动作枚举、交互前向仿真、代价评价与最优行为选择接口。
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/mobil/mobil_model.h"
#include "common/state/state.h"
#include "eudm_config.pb.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/map_interface.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/**
 * @brief 基于 DCP 动作脚本和多车交互前向仿真的 EUDM 行为规划器。
 *
 * 每周期从语义地图读取自车与关键周车，对每条纵横向离散动作序列并行前向仿真，执行严格
 * 碰撞/RSS 检查并累计效率、安全、导航代价，最后选择最低代价序列。EudmManager 负责注入
 * 地图、任务和正在执行的动作，并读取本类保存的完整候选结果。
 */
class EudmPlanner : public Planner {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;
  using Cfg = planning::eudm::Config;
  using LaneChangeInfo = planning::eudm::LaneChangeInfo;

  /// 一条动作脚本在场景级呈现的横向时序模式。
  enum class LatSimMode {
    kAlwaysLaneKeep = 0,
    kKeepThenChange,
    kAlwaysLaneChange,
    kChangeThenCancel
  };

  /// 自车前向仿真上下文，按场景、动作层和积分步三个频率更新。
  struct ForwardSimEgoAgent {
    // 整个仿真过程使用的横向前车/邻车搜索范围。
    decimal_t lat_range;

    // 场景级：纵横向动力学参数和整条动作序列的横向模式。
    OnLaneForwardSimulation::Param sim_param;

    LatSimMode seq_lat_mode;
    common::LateralBehavior lat_behavior_longterm{LateralBehavior::kUndefined};
    common::LateralBehavior seq_lat_behavior;
    bool is_cancel_behavior;
    decimal_t operation_at_seconds{0.0};

    // 动作层级：当前离散行为及当前/目标/长期参考 Lane 与坐标变换。
    common::LongitudinalBehavior lon_behavior{LongitudinalBehavior::kMaintain};
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};

    common::Lane current_lane;
    common::StateTransformer current_stf;
    common::Lane target_lane;
    common::StateTransformer target_stf;
    common::Lane longterm_lane;
    common::StateTransformer longterm_stf;

    // 目标换道间隙的前/后车 ID；每层固定，车辆状态随积分步变化。
    Vec2i target_gap_ids;

    // 积分步级：自车最新预测状态。
    common::Vehicle vehicle;
  };

  /// 单个周车的动力学参数、预测横向行为、参考 Lane 和最新预测状态。
  struct ForwardSimAgent {
    int id = kInvalidAgentId;
    common::Vehicle vehicle;

    // 纵向 IDM 和运动学传播参数。
    OnLaneForwardSimulation::Param sim_param;

    // 横向行为概率、当前采用行为及其参考 Lane。
    common::ProbDistOfLatBehaviors lat_probs;
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};

    common::Lane lane;
    common::StateTransformer stf;

    // 搜索前车时使用的横向命中半径。
    decimal_t lat_range;
  };

  /// 以车辆 ID 索引的周车前向仿真上下文集合。
  struct ForwardSimAgentSet {
    std::unordered_map<int, ForwardSimAgent> forward_sim_agents;
  };

  /// 自车速度偏差和前车阻塞造成的效率代价。
  struct EfficiencyCost {
    decimal_t ego_to_desired_vel = 0.0;
    decimal_t leading_to_desired_vel = 0.0;
    decimal_t ave() const {
      return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
    }
  };

  /// RSS 风险与占用/禁换 Lane 造成的安全代价。
  struct SafetyCost {
    decimal_t rss = 0.0;
    decimal_t occu_lane = 0.0;
    decimal_t ave() const { return (rss + occu_lane) / 2.0; }
  };

  /// 换道、取消和用户推荐相关的导航偏好代价。
  struct NavigationCost {
    decimal_t lane_change_preference = 0.0;
    decimal_t ave() const { return lane_change_preference; }
  };

  /// 单个动作层的分项代价、轨迹有效上界和时间/折扣权重。
  struct CostStructure {
    // 本层结束时对应的累计轨迹样本上界。
    int valid_sample_index_ub;
    // 效率、安全、导航三类分项。
    EfficiencyCost efficiency;
    SafetyCost safety;
    NavigationCost navigation;
    // 动作持续时间乘场景折扣后的聚合权重。
    decimal_t weight = 1.0;
    /// 返回三个分项均值之和再乘权重的层代价。
    decimal_t ave() const {
      return (efficiency.ave() + safety.ave() + navigation.ave()) * weight;
    }

    /// 输出效率、安全和导航分项，供候选诊断日志使用。
    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructure& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(efficiency: "
         << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
         << cost.efficiency.leading_to_desired_vel << "), safety: ("
         << cost.safety.rss << "," << cost.safety.occu_lane
         << "), navigation: " << cost.navigation.lane_change_preference << ")";
      return os;
    }
  };

  /// 返回规划器日志名称。
  std::string Name() override;

  /// 从 protobuf 文本配置初始化动作树、前向仿真参数和三组 RSS 参数。
  ErrorType Init(const std::string config) override;

  /// 执行一个完整规划周期并保存胜出候选和全部诊断结果。
  ErrorType RunOnce() override;

  /// 注入非拥有的语义地图接口指针。
  void set_map_interface(EudmPlannerMapItf* itf);
  /// 设置非负用户期望速度。
  void set_desired_velocity(const decimal_t desired_vel);

  /// 设置本周期换道禁用、安全、实线和推荐信息。
  void set_lane_change_info(const LaneChangeInfo& lc_info);

  /// 对全部 DCP 候选并行仿真、汇总有效性并选择最低代价候选。
  ErrorType RunEudm();

  /// 返回语义行为；当前仅有声明，源码中没有对应定义。
  Behavior behavior() const;

  /// 按值返回胜出候选动作缓存；当前实现没有给 winner_action_seq_ 赋值。
  std::vector<DcpAction> winner_action_seq() const {
    return winner_action_seq_;
  }

  /// 返回当前用户期望速度。
  decimal_t desired_velocity() const;

  /// 按值返回全部候选的自车预测轨迹。
  vec_E<vec_E<common::Vehicle>> forward_trajs() const { return forward_trajs_; }

  /// 返回最低代价候选索引。
  int winner_id() const;

  /// 返回最近一次 RunOnce 总耗时，单位由 TicToc 约定。
  decimal_t time_cost() const;

  /// 把内部整数仿真状态转换为 bool 向量并返回。
  std::vector<bool> sim_res() const {
    std::vector<bool> ret;
    for (auto& r : sim_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }

  /// 把内部整数风险状态转换为 bool 向量并返回。
  std::vector<bool> risky_res() const {
    std::vector<bool> ret;
    for (auto& r : risky_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }
  /// 按值返回每个候选的诊断字符串。
  std::vector<std::string> sim_info() const { return sim_info_; }
  /// 按值返回每个候选的最终总代价。
  std::vector<decimal_t> final_cost() const { return final_cost_; }
  /// 按值返回每个候选逐动作层的代价轨迹。
  std::vector<std::vector<CostStructure>> progress_cost() const {
    return progress_cost_;
  }
  /// 按值返回每个候选的末端代价容器。
  std::vector<CostStructure> tail_cost() const { return tail_cost_; }
  /// 按值返回每个候选逐层采用的横向行为。
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }
  /// 按值返回每个候选逐层采用的纵向行为。
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }
  /// 按值返回每个候选中以周车 ID 索引的预测轨迹。
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs()
      const {
    return surround_trajs_;
  }
  /// 返回本周期规划起点的自车状态副本。
  common::State plan_state() { return ego_vehicle_.state(); }
  /// 从 DCP tree 按值返回全部候选动作脚本。
  std::vector<std::vector<DcpAction>> action_script() {
    return dcp_tree_ptr_->action_script();
  }

  /// 返回只读 protobuf 配置引用。
  const Cfg& cfg() const { return cfg_; }

  /// 返回非拥有地图接口指针。
  EudmPlannerMapItf* map_itf() const;

  /// 更新正在执行的 DCP 动作、重建候选并刷新总仿真时域。
  void UpdateDcpTree(const DcpAction& ongoing_action);

  /// 识别动作序列的首次换道时刻、长期横向行为和是否包含取消。
  ErrorType ClassifyActionSeq(const std::vector<DcpAction>& action_seq,
                              decimal_t* operation_at_seconds,
                              common::LateralBehavior* lat_behavior,
                              bool* is_cancel_operation) const;

 private:
  /// 解析 protobuf text 配置到 cfg_。
  ErrorType ReadConfig(const std::string config_path);

  /// 把单车 ForwardSimDetail 配置映射为传播器参数。
  ErrorType GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                         OnLaneForwardSimulation::Param* sim_param);

  /// 按源 Lane 和横向行为收集相邻 Lane 及其后继候选。
  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                 const LateralBehavior& beh,
                                 std::vector<int>* candidate_lane_ids) const;
  /// 更新缓存的自车 Lane ID；当前潜在 Lane 列表刷新代码已被注释。
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  /// 根据当前位置 Lane 与缓存候选的关系推断保持/左换/右换。
  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                   LateralBehavior* behavior_by_lane_id);

  /// 按 Lane ID 推断结果更新自车横向行为；当前仅声明且未定义。
  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  /// 把 DCP 纵横向枚举转换为 common 行为枚举。
  ErrorType TranslateDcpActionToLonLatBehavior(const DcpAction& action,
                                               LateralBehavior* lat,
                                                LongitudinalBehavior* lon) const;

  /// 把关键语义周车转换为可逐步传播的 ForwardSimAgentSet。
  ErrorType GetSurroundingForwardSimAgents(
      const common::SemanticVehicleSet& surrounding_semantic_vehicles,
      ForwardSimAgentSet* forward_sim_agents) const;

  // 仿真控制链：候选序列 -> 场景 -> 单动作层 -> 单积分步。
  /// 在线程入口中仿真一条候选序列，并把默认子场景结果写回共享容器槽位。
  ErrorType SimulateActionSequence(
      const common::Vehicle& ego_vehicle,
      const ForwardSimAgentSet& surrounding_fsagents,
      const std::vector<DcpAction>& action_seq, const int& seq_id);

  /// 逐层仿真一条动作序列，执行安全检查、代价计算并累计多车轨迹。
  ErrorType SimulateScenario(
      const common::Vehicle& ego_vehicle,
      const ForwardSimAgentSet& surrounding_fsagents,
      const std::vector<DcpAction>& action_seq, const int& seq_id,
      const int& sub_seq_id, std::vector<int>* sub_sim_res,
      std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
      std::vector<std::vector<CostStructure>>* sub_progress_cost,
      std::vector<CostStructure>* sub_tail_cost,
      vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
      std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
      std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
          sub_surround_trajs);

  /// 按配置积分步长传播一个 DCP 动作层内的自车和全部周车。
  ErrorType SimulateSingleAction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
      const ForwardSimAgentSet& surrounding_fsagents_this_layer,
      vec_E<common::Vehicle>* ego_traj_multisteps,
      std::unordered_map<int, vec_E<common::Vehicle>>*
          surround_trajs_multisteps);

  // 评价函数：层代价、严格碰撞、RSS 风险和候选总分。
  /// 计算单动作层的效率、安全和导航代价及风险车辆集合。
  ErrorType CostFunction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
      const ForwardSimAgentSet& other_fsagent,
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool verbose, CostStructure* cost, bool* is_risky,
      std::set<int>* risky_ids);

  /// 对齐检查自车/周车轨迹并逐样本执行膨胀车身碰撞检测。
  ErrorType StrictSafetyCheck(
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool* is_safe, int* collided_id);

  /// 对一对等长轨迹逐样本执行 RSS 检查并累计速度违反代价。
  ErrorType EvaluateSafetyStatus(const vec_E<common::Vehicle>& traj_a,
                                 const vec_E<common::Vehicle>& traj_b,
                                 decimal_t* cost, bool* is_rss_safe,
                                 int* risky_id);
  /// 汇总候选逐层代价和末端代价为一个标量分数。
  ErrorType EvaluateSinglePolicyTrajs(
      const std::vector<CostStructure>& progress_cost,
      const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
      decimal_t* score);

  /// 遍历成功候选并返回最低代价索引和分数。
  ErrorType EvaluateMultiThreadSimResults(int* winner_id,
                                           decimal_t* winner_cost);

  // 仿真配置与传播辅助函数。
  /// 从整条动作序列初始化场景级横向模式、纵向期望速度和自车参数。
  ErrorType UpdateSimSetupForScenario(const std::vector<DcpAction>& action_seq,
                                       ForwardSimEgoAgent* ego_fsagent) const;

  /// 为当前动作层更新行为、三条参考 Lane、换道目标间隙和层级 RSS 门限。
  ErrorType UpdateSimSetupForLayer(const DcpAction& action,
                                   const ForwardSimAgentSet& other_fsagent,
                                    ForwardSimEgoAgent* ego_fsagent) const;

  /// 只更新自车仿真上下文中的纵横向行为枚举。
  ErrorType UpdateEgoBehaviorsUsingAction(
      const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const;

  /// 通过当前位置是否进入目标 Lane 候选集合判断横向动作是否完成。
  bool CheckIfLateralActionFinished(const common::State& cur_state,
                                    const int& action_ref_lane_id,
                                    const LateralBehavior& lat_behavior,
                                    int* current_lane_id) const;

  /// 换道提前完成后重写剩余横向动作，保持原脚本的取消/回切语义。
  ErrorType UpdateLateralActionSequence(
      const int cur_idx, std::vector<DcpAction>* action_seq) const;

  /// 清空并按候选数量预分配所有线程共享结果容器。
  ErrorType PrepareMultiThreadContainers(const int n_sequence);

  /// 把动作时长拆成固定 step 和一个余数积分步。
  ErrorType GetSimTimeSteps(const DcpAction& action,
                             std::vector<decimal_t>* dt_steps) const;

  /// 在当前交通快照中传播自车一个积分步。
  ErrorType EgoAgentForwardSim(const ForwardSimEgoAgent& ego_fsagent,
                               const common::VehicleSet& all_sim_vehicles,
                               const decimal_t& sim_time_step,
                               common::State* state_out) const;

  /// 在当前交通快照中传播单个周车一个积分步。
  ErrorType SurroundingAgentForwardSim(
      const ForwardSimAgent& fsagent,
      const common::VehicleSet& all_sim_vehicles,
      const decimal_t& sim_time_step, common::State* state_out) const;

  // 地图与候选动作树。
  /// 非拥有地图接口，由 EudmManager 注入并保证生命周期。
  EudmPlannerMapItf* map_itf_{nullptr};
  /// Init 中动态创建的 DCP tree；当前没有类内初值或析构释放。
  DcpTree* dcp_tree_ptr_;
  // 配置与本周期输入/派生状态。
  /// protobuf 配置及换道任务信息。
  Cfg cfg_;
  LaneChangeInfo lc_info_;
  /// 用户期望速度和当前 DCP 候选总时域。
  decimal_t desired_velocity_{5.0};
  decimal_t sim_time_total_ = 0.0;
  /// 在仿真前因直接左右反向切换而删除的候选索引。
  std::set<int> pre_deleted_seq_ids_;
  /// 自车当前 Lane 及保持/左换/右换可能到达的 Lane 缓存。
  int ego_lane_id_{kInvalidLaneId};
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  /// RSS 检查使用的长参考 Lane 和对应 Frenet 变换。
  common::Lane rss_lane_;
  common::StateTransformer rss_stf_;
  /// 常规 RSS 以及自车作为前车/后车时的严格参数。
  common::RssChecker::RssConfig rss_config_;
  common::RssChecker::RssConfig rss_config_strict_as_front_;
  common::RssChecker::RssConfig rss_config_strict_as_rear_;

  /// 自车与周车的基础前向传播参数。
  OnLaneForwardSimulation::Param ego_sim_param_;
  OnLaneForwardSimulation::Param agent_sim_param_;

  /// 本周期时间戳、自车 ID 和规划起点车辆；时间戳/ID 当前无类内初值。
  decimal_t time_stamp_;
  int ego_id_;
  common::Vehicle ego_vehicle_;

  // 最近一次规划结果及全部候选诊断。
  /// 胜出候选索引、分数和动作序列缓存；动作序列缓存当前未被写入。
  int winner_id_ = 0;
  decimal_t winner_score_ = 0.0;
  std::vector<DcpAction> winner_action_seq_;
  /// 每个候选的仿真有效、RSS 风险、诊断和最终代价。
  std::vector<int> sim_res_;
  std::vector<int> risky_res_;
  std::vector<std::string> sim_info_;
  std::vector<decimal_t> final_cost_;
  /// 每个候选的逐层代价与末端代价；末端代价当前只保留默认值。
  std::vector<std::vector<CostStructure>> progress_cost_;
  std::vector<CostStructure> tail_cost_;
  /// 每个候选的自车轨迹、逐层行为和全部周车轨迹。
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
  /// 最近一次 RunOnce 的总耗时。
  decimal_t time_cost_ = 0.0;
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
