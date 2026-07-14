/**
 * @file eudm_manager.h
 * @brief EUDM 跨周期重规划、换道 HMI 状态机和结果快照管理接口。
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_

#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"

namespace planning {

/**
 * @brief 在无跨周期状态的 EudmPlanner 外管理任务约束、动作续接和换道上下文。
 *
 * Manager 每周期注入地图与 Task，从上次胜出脚本恢复 ongoing action，驱动 stick/active 换道
 * 状态机，运行 planner 后保存全部候选快照，并按当前换道上下文重选最终输出。
 */
class EudmManager {
 public:
  using DcpLatAction = planning::DcpTree::DcpLatAction;
  using DcpLonAction = planning::DcpTree::DcpLonAction;
  using DcpAction = planning::DcpTree::DcpAction;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using CostStructure = planning::EudmPlanner::CostStructure;

  /// 换道由用户拨杆直接触发，或由规划结果连续积累后主动触发。
  enum class LaneChangeTriggerType { kStick = 0, kActive };

  /// 上次最终选中脚本及其起始时间，用于恢复当前仍在执行的 DCP 动作。
  struct ReplanningContext {
    /// 是否存在可续接脚本。
    bool is_valid = false;
    /// 脚本开始执行的绝对时间和动作序列。
    decimal_t seq_start_time;
    std::vector<DcpAction> action_seq;
  };

  /// 单帧主动换道请求，用于跨帧检查方向、Lane 和操作时刻一致性。
  struct ActivateLaneChangeRequest {
    /// 请求产生时间、计划操作时间和请求时自车 Lane。
    decimal_t trigger_time;
    decimal_t desired_operation_time;
    int ego_lane_id;
    /// 请求的换道方向。
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  /// 连续主动请求达到阈值后生成、仅供下一周期消费的换道提案。
  struct LaneChangeProposal {
    /// 提案是否可消费及其生成时刻。
    bool valid = false;
    decimal_t trigger_time = 0.0;
    /// 相对操作延迟、提案 Lane 和方向。
    decimal_t operation_at_seconds = 0.0;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  /// 当前 stick/active 换道任务的生命周期和计划执行时刻。
  struct LaneChangeContext {
    /// completed=false 表示换道任务正在等待或执行。
    bool completed = true;
    /// 禁换解除或条件满足后是否自动触发缓存请求。
    bool trigger_when_appropriate = false;
    /// 触发时间、期望绝对操作时间和触发时 Lane。
    decimal_t trigger_time = 0.0;
    decimal_t desired_operation_time = 0.0;
    int ego_lane_id = 0;
    /// 换道方向和触发来源。
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
    LaneChangeTriggerType type;
  };

  /// 单次成功规划的输入状态、全部候选诊断和最终重选结果快照。
  struct Snapshot {
    /// 只有完整 Run 成功并保存后才为 true。
    bool valid = false;
    /// planner 原始 winner 与 manager 上下文重选 winner。
    int original_winner_id;
    int processed_winner_id;
    /// 规划起点和全部 DCP/仿真/代价结果。
    common::State plan_state;
    std::vector<std::vector<DcpAction>> action_script;
    std::vector<bool> sim_res;
    std::vector<bool> risky_res;
    std::vector<std::string> sim_info;
    std::vector<decimal_t> final_cost;
    std::vector<std::vector<CostStructure>> progress_cost;
    std::vector<CostStructure> tail_cost;
    vec_E<vec_E<common::Vehicle>> forward_trajs;
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs;
    /// 为最终重选候选拟合的高质量参考 Lane。
    common::Lane ref_lane;

    /// 语义地图规划时间戳和 planner 计算耗时。
    double plan_stamp = 0.0;
    double time_cost = 0.0;
  };

  /// 默认构造 manager 及其 planner/adapter/状态缓存。
  EudmManager() {}

  /// 初始化日志、planner 配置、地图接口绑定和 manager 工作频率。
  void Init(const std::string& config_path, const decimal_t work_rate);

  /// 执行 Prepare、planner RunOnce、快照、上下文重选和状态更新完整周期。
  ErrorType Run(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  /// 使动作续接上下文失效；当前不会清空快照和换道状态。
  void Reset();

  /// 从最后成功快照的 processed winner 构造下游 SemanticBehavior。
  void ConstructBehavior(common::SemanticBehavior* behavior);

  /// 返回内部 EudmPlanner 可变引用。
  EudmPlanner& planner();

  /// 返回最后快照的原始最低代价候选索引。
  int original_winner_id() const { return last_snapshot_.original_winner_id; }
  /// 返回经换道上下文重选后的候选索引。
  int processed_winner_id() const { return last_snapshot_.processed_winner_id; }
  /// 返回当前 SemanticMapManager 共享指针。
  std::shared_ptr<semantic_map_manager::SemanticMapManager> map() {
    return map_adapter_.map();
  }

 private:
  /// 把 stamp+delta 向上对齐到下一个 DCP layer 决策点。
  decimal_t GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                           const decimal_t& delta);

  /// 判断当前是否适合执行指定换道；现实现被编译期开关固定为 true。
  bool IsTriggerAppropriate(const LateralBehavior& lat);

  /// 注入地图、恢复 ongoing action、更新换道状态并设置 planner 本周期输入。
  ErrorType Prepare(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  /// 根据上一参考 Lane 曲率和用户速度计算本周期参考速度。
  ErrorType EvaluateReferenceVelocity(const planning::eudm::Task& task,
                                       decimal_t* ref_vel);

  /// 从上次脚本和经过时间提取当前仍在执行的动作及剩余时长。
  bool GetReplanDesiredAction(const decimal_t current_time,
                               DcpAction* desired_action);

  /// 深拷贝 planner 全部结果和地图时间戳到 Snapshot。
  void SaveSnapshot(Snapshot* snapshot);

  /// 在符合当前换道上下文的成功候选中重新选择最低代价序列。
  ErrorType ReselectByContext(const decimal_t stamp, const Snapshot& snapshot,
                               int* new_seq_id);

  /// 根据控制权、拨杆、禁换信号、提案和超时推进换道状态机。
  void UpdateLaneChangeContextByTask(const decimal_t stamp,
                                      const planning::eudm::Task& task);

  /// 从连续多帧原始 winner 中积累并生成主动换道提案。
  ErrorType GenerateLaneChangeProposal(const decimal_t& stamp,
                                        const planning::eudm::Task& task);

  /// 核心 planner 和把 SemanticMapManager 暴露为 planner map interface 的适配器。
  EudmPlanner bp_;
  EudmPlannerMapAdapter map_adapter_;
  /// manager 期望运行频率。
  decimal_t work_rate_{20.0};

  /// 当前自车 Lane ID 和上一最终脚本的动作续接上下文。
  int ego_lane_id_;
  ReplanningContext context_;
  /// 最近成功规划快照和上一周期用户任务。
  Snapshot last_snapshot_;
  planning::eudm::Task last_task_;
  /// 当前换道状态、上一主动提案和正在积累的多帧请求。
  LaneChangeContext lc_context_;
  LaneChangeProposal last_lc_proposal_;
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_;
};

}  // namespace planning

#endif
