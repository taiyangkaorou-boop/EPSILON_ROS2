#include "behavior_planner/behavior_planner.h"

namespace planning {

// 返回通用行为规划器名称。
std::string BehaviorPlanner::Name() {
  return std::string("Generic behavior planner");
}

// 创建 RoutePlanner 并初始化输出速度；config 当前未解析。
ErrorType BehaviorPlanner::Init(const std::string config) {
  // 重复 Init 会覆盖裸指针而不释放旧对象。
  p_route_planner_ = new planning::RoutePlanner();
  behavior_.actual_desired_velocity = 0.0;
  return kSuccess;
}

// 调用多行为决策并把 winner、全部候选 rollout 和周车轨迹写入 SemanticBehavior。
ErrorType BehaviorPlanner::RunMpdm() {
  TicToc timer;
  LateralBehavior mpdm_behavior;
  decimal_t mpdm_desired_velocity;
  if (MultiBehaviorJudge(behavior_.actual_desired_velocity, &mpdm_behavior,
                         &mpdm_desired_velocity) == kSuccess) {
    behavior_.lat_behavior = mpdm_behavior;
    behavior_.actual_desired_velocity = mpdm_desired_velocity;
    behavior_.forward_trajs = forward_trajs_;
    behavior_.forward_behaviors = forward_behaviors_;
    behavior_.surround_trajs = surround_trajs_;

    printf("[MPDM]MPDM desired velocity %lf in %lf.\n",
           behavior_.actual_desired_velocity, reference_desired_velocity_);
    printf("[MPDM]Time multi behavior judged in %lf ms.\n", timer.toc());
  } else {
    printf("[MPDM]MPDM failed.\n");
    printf("[MPDM]Time multi behavior judged in %lf ms.\n", timer.toc());
    return kWrongStatus;
  }
  return kSuccess;
}

// 以随机扩展模式刷新导航路径；底层查询和 RunOnce 失败均未向上返回。
ErrorType BehaviorPlanner::RunRoutePlanner(const int nearest_lane_id) {
  TicToc timer_rp;
  p_route_planner_->set_navi_mode(RoutePlanner::NaviMode::kRandomExpansion);
  if (!p_route_planner_->if_get_lane_net()) {
    common::LaneNet whole_lane_net;
    map_itf_->GetWholeLaneNet(&whole_lane_net);
    p_route_planner_->set_lane_net(whole_lane_net);
  }
  State ego_state;
  map_itf_->GetEgoState(&ego_state);
  p_route_planner_->set_ego_state(ego_state);
  p_route_planner_->set_nearest_lane_id(nearest_lane_id);
  if (p_route_planner_->RunOnce() == kSuccess) {
  }
  return kSuccess;
}

// 完成单周期车道归属、候选行为、MPDM winner 和参考 Lane 更新。
ErrorType BehaviorPlanner::RunOnce() {
  // 先使用 RoutePlanner 当前导航路径确定自车最近 Lane。
  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(p_route_planner_->navi_path(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    printf("[BP RunOnce]Err - Ego not on lane.\n");
    return kWrongStatus;
  }

  common::Vehicle ego_vehicle;
  if (map_itf_->GetEgoVehicle(&ego_vehicle) != kSuccess) {
    printf("[MPDM]fail to get ego vehicle.\n");
    return kWrongStatus;
  }
  ego_id_ = ego_vehicle.id();

  // 仿真状态模式下刷新随机扩展导航路径；返回码被忽略。
  if (use_sim_state_) {
    RunRoutePlanner(ego_lane_id_by_pos);
  }

  // 首周期初始化内部 Lane ID；这里使用 Agent 无效常量而非 Lane 无效常量。
  if (ego_lane_id_ == kInvalidAgentId) {
    UpdateEgoLaneId(ego_lane_id_by_pos);
  }

  LateralBehavior behavior_by_lane_id;
  if (JudgeBehaviorByLaneId(ego_lane_id_by_pos, &behavior_by_lane_id) !=
      kSuccess) {
    printf("[RunOnce]fail to judge behavior by lane id!\n");
    return kWrongStatus;
  }

  UpdateEgoLaneId(ego_lane_id_by_pos);
  printf("[MPDM]ego lane id: %d.\n", ego_lane_id_);

  if (UpdateEgoBehavior(behavior_by_lane_id) != kSuccess) {
    printf("[RunOnce]fail to update ego behavior!\n");
    return kWrongStatus;
  }

  if (behavior_.lat_behavior == common::LateralBehavior::kUndefined) {
    // 未定义行为临时回退为保持车道。
    behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
  }

  behavior_.actual_desired_velocity = user_desired_velocity_;
  if (autonomous_level_ >= 3) {
    TicToc timer;
    planning::MultiModalForward::ParamLookUp(aggressive_level_, &sim_param_);
    if (RunMpdm() != kSuccess) {
      printf("[Summary]Mpdm failed: %lf ms.\n", timer.toc());
      return kWrongStatus;
    } else {
    }
    printf("[Summary]Mpdm time cost: %lf ms.\n", timer.toc());
  }

  if (ConstructReferenceLane(behavior_.lat_behavior, &behavior_.ref_lane) !=
      kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// 枚举 LK/LCL/LCR，补全周车参考 Lane，执行 rollout 并选择最低代价策略。
ErrorType BehaviorPlanner::MultiBehaviorJudge(
    const decimal_t previous_desired_vel, LateralBehavior* mpdm_behavior,
    decimal_t* mpdm_desired_velocity) {
  // 获取语义关键周车和自车快照。
  common::SemanticVehicleSet semantic_vehicle_set;
  if (map_itf_->GetKeySemanticVehicles(&semantic_vehicle_set) != kSuccess) {
    printf("[MPDM]fail to get key vehicles.\n");
    return kWrongStatus;
  }

  common::Vehicle ego_vehicle;
  if (map_itf_->GetEgoVehicle(&ego_vehicle) != kSuccess) {
    printf("[MPDM]fail to get ego vehicle.\n");
    return kWrongStatus;
  }

  std::cout << "[BehaviorPlanner]" << ego_vehicle.id()
            << "\tsemantic_vehicle_set num:"
            << semantic_vehicle_set.semantic_vehicles.size() << std::endl;

  // 清空上一周期调试/输出缓存。
  forward_trajs_.clear();
  forward_behaviors_.clear();
  surround_trajs_.clear();

  // LK 始终候选，左右换道取决于前序 Lane 拓扑缓存是否非空。
  std::vector<LateralBehavior> potential_behaviors{
      common::LateralBehavior::kLaneKeeping};
  if (!potential_lcl_lane_ids_.empty())
    potential_behaviors.push_back(common::LateralBehavior::kLaneChangeLeft);
  if (!potential_lcr_lane_ids_.empty())
    potential_behaviors.push_back(common::LateralBehavior::kLaneChangeRight);

  // 按每辆周车的预测横向行为构造 rollout 参考 Lane。
  const decimal_t max_backward_len = 10.0;
  for (auto it = semantic_vehicle_set.semantic_vehicles.begin();
       it != semantic_vehicle_set.semantic_vehicles.end(); ++it) {
    common::LateralBehavior lat_behavior;
    if (map_itf_->GetPredictedBehavior(it->second.vehicle.id(),
                                       &lat_behavior) != kSuccess) {
      lat_behavior = common::LateralBehavior::kLaneKeeping;
    }
    decimal_t forward_lane_len =
        std::max(it->second.vehicle.state().velocity * 10.0, 50.0);
    common::Lane ref_lane;
    if (map_itf_->GetRefLaneForStateByBehavior(
            it->second.vehicle.state(), std::vector<int>(), lat_behavior,
            forward_lane_len, max_backward_len, false, &ref_lane) == kSuccess) {
      it->second.lane = ref_lane;
    }
  }

  // 对每个候选自车行为独立执行多车前向仿真，失败候选直接丢弃。
  std::vector<LateralBehavior> valid_behaviors;
  vec_E<vec_E<common::Vehicle>> valid_forward_trajs;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> valid_surround_trajs;
  int num_available_behaviors = static_cast<int>(potential_behaviors.size());

  for (int i = 0; i < num_available_behaviors; i++) {
    vec_E<common::Vehicle> traj;
    std::unordered_map<int, vec_E<common::Vehicle>> sur_trajs;
    if (SimulateEgoBehavior(ego_vehicle, potential_behaviors[i],
                            semantic_vehicle_set, &traj,
                            &sur_trajs) != kSuccess) {
      printf("[MPDM]fail to sim %d forward.\n",
             static_cast<int>(potential_behaviors[i]));
      continue;
    }
    valid_behaviors.push_back(potential_behaviors[i]);
    valid_forward_trajs.push_back(traj);
    valid_surround_trajs.push_back(sur_trajs);
  }
  // 缓存所有有效候选，供输出语义和可视化使用。
  forward_behaviors_ = valid_behaviors;
  forward_trajs_ = valid_forward_trajs;
  surround_trajs_ = valid_surround_trajs;

  // 至少需要一个有效候选，再进入统一代价评估。
  int num_valid_behaviors = static_cast<int>(valid_behaviors.size());
  if (num_valid_behaviors < 1) {
    printf("[MPDM]No valid behaviors.\n");
    return kWrongStatus;
  }

  decimal_t winner_score, winner_desired_vel;
  LateralBehavior winner_behavior;
  vec_E<common::Vehicle> winner_forward_traj;
  if (EvaluateMultiPolicyTrajs(valid_behaviors, valid_forward_trajs,
                               valid_surround_trajs, &winner_behavior,
                               &winner_forward_traj, &winner_score,
                               &winner_desired_vel) != kSuccess) {
    printf("[MPDM]fail to evaluate multiple policy trajs.\n");
    return kWrongStatus;
  }
  // 速度命令相对当前车速最多跳变 5 m/s；未额外截断为非负。
  const decimal_t max_vel_cmd_gap = 5.0;
  if (fabs(winner_desired_vel - ego_vehicle.state().velocity) >
      max_vel_cmd_gap) {
    if (winner_desired_vel > ego_vehicle.state().velocity) {
      winner_desired_vel = ego_vehicle.state().velocity + max_vel_cmd_gap;
    } else {
      winner_desired_vel = ego_vehicle.state().velocity - max_vel_cmd_gap;
    }
  }

  if (lock_to_hmi_) {
    // HMI 行为存在于有效候选时只覆盖横向 winner，速度仍使用原 MPDM winner 的结果。
    auto it = std::find(forward_behaviors_.begin(), forward_behaviors_.end(),
                        hmi_behavior_);
    if (it != forward_behaviors_.end()) {
      *mpdm_behavior = hmi_behavior_;
    } else {
      *mpdm_behavior = winner_behavior;
    }
  } else {
    *mpdm_behavior = winner_behavior;
  }
  // previous_desired_vel 当前未参与任何平滑或迟滞计算。
  *mpdm_desired_velocity = winner_desired_vel;
  return kSuccess;
}

ErrorType BehaviorPlanner::OpenloopSimForward(
    const common::SemanticVehicle& ego_semantic_vehicle,
    const common::SemanticVehicleSet& agent_vehicles,
    vec_E<common::Vehicle>* traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // 初始化输出：自车和每辆周车的轨迹都以当前时刻状态作为第一个采样点。
  traj->clear();
  traj->push_back(ego_semantic_vehicle.vehicle);
  surround_trajs->clear();
  for (const auto &v : agent_vehicles.semantic_vehicles) {
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
    surround_trajs->at(v.first).push_back(v.second.vehicle);
  }

  // 仿真步数按 horizon/resolution 向下取整；临时集合保存逐步推进后的周车状态。
  int num_steps_forward = static_cast<int>(sim_horizon_ / sim_resolution_);
  common::Vehicle cur_ego_vehicle = ego_semantic_vehicle.vehicle;
  common::SemanticVehicleSet semantic_vehicle_set_tmp = agent_vehicles;
  common::State ego_state;
  for (int i = 0; i < num_steps_forward; i++) {
    // 开环降级不查询前车：自车按参考期望速度沿候选参考车道独立传播一步。
    sim_param_.idm_param.kDesiredVelocity = reference_desired_velocity_;
    if (planning::OnLaneForwardSimulation::PropagateOnce(
            common::StateTransformer(ego_semantic_vehicle.lane),
            cur_ego_vehicle, common::Vehicle(), sim_resolution_, sim_param_,
            &ego_state) != kSuccess) {
      return kWrongStatus;
    }

    // 周车也不考虑相互作用，并保持各自初始速度为期望速度。先缓存所有下一状态，
    // 保证同一仿真步内每辆车都从相同时间切片出发，不受遍历顺序影响。
    std::unordered_map<int, State> state_cache;
    for (auto& v : semantic_vehicle_set_tmp.semantic_vehicles) {
      decimal_t desired_vel =
          agent_vehicles.semantic_vehicles.at(v.first).vehicle.state().velocity;
      sim_param_.idm_param.kDesiredVelocity = desired_vel;
      common::State agent_state;
      if (planning::OnLaneForwardSimulation::PropagateOnce(
              common::StateTransformer(v.second.lane), v.second.vehicle,
              common::Vehicle(), sim_resolution_, sim_param_,
              &agent_state) != kSuccess) {
        return kWrongStatus;
      }
      state_cache.insert(std::make_pair(v.first, agent_state));
    }

    // 仅使用地图接口筛查自车下一状态；碰撞时终止本候选行为的开环预测。
    bool is_collision = false;
    map_itf_->CheckIfCollision(ego_semantic_vehicle.vehicle.param(), ego_state,
                               &is_collision);
    if (is_collision) return kWrongStatus;

    // 同步提交本步缓存，并把更新后的自车、周车状态追加到各自轨迹。
    cur_ego_vehicle.set_state(ego_state);
    for (auto& s : state_cache) {
      semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle.set_state(
          s.second);
      surround_trajs->at(s.first).push_back(
          semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
    }
    traj->push_back(cur_ego_vehicle);
  }
  return kSuccess;
}

ErrorType BehaviorPlanner::SimulateEgoBehavior(
    const common::Vehicle& ego_vehicle, const LateralBehavior& ego_behavior,
    const common::SemanticVehicleSet& semantic_vehicle_set,
    vec_E<common::Vehicle>* traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  const decimal_t max_backward_len = 10.0;
  // 前向参考车道至少覆盖 50 m，高速时按当前速度的 10 倍延长；后向固定覆盖 10 m。
  decimal_t forward_lane_len =
      std::max(ego_vehicle.state().velocity * 10.0, 50.0);
  common::Lane ego_reflane;
  // 将候选 LK/LCL/LCR 行为映射为本次 rollout 使用的自车参考车道。
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle.state(), p_route_planner_->navi_path(), ego_behavior,
          forward_lane_len, max_backward_len, false,
          &ego_reflane) != kSuccess) {
    printf("[MPDM]fail to get ego reference lane.\n");
    return kWrongStatus;
  }

  // 组合车辆本体与候选参考车道，形成多车仿真接口所需的语义自车。
  common::SemanticVehicle ego_semantic_vehicle;
  {
    ego_semantic_vehicle.vehicle = ego_vehicle;
    ego_semantic_vehicle.lane = ego_reflane;
  }

  // 在周车集合副本中加入自车；原始输入保留给后续开环降级路径使用。
  common::SemanticVehicleSet semantic_vehicle_set_tmp = semantic_vehicle_set;
  semantic_vehicle_set_tmp.semantic_vehicles.insert(
      std::make_pair(ego_vehicle.id(), ego_semantic_vehicle));

  // 首选包含跟驰交互的同步多车 rollout，以获得自车与周车的联合预测轨迹。
  printf("[MPDM]simulating behavior %d.\n", static_cast<int>(ego_behavior));
  if (MultiAgentSimForward(ego_vehicle.id(), semantic_vehicle_set_tmp, traj,
                           surround_trajs) != kSuccess) {
    printf("[MPDM]multi agent forward under %d failed.\n",
           static_cast<int>(ego_behavior));
    // 多车仿真失败时退化为各车辆互不响应的固定车道开环传播。
    if (OpenloopSimForward(ego_semantic_vehicle, semantic_vehicle_set, traj,
                           surround_trajs) != kSuccess) {
      printf("[MPDM]open loop forward under %d failed.\n",
             static_cast<int>(ego_behavior));
      return kWrongStatus;
    }
  }
  printf("[MPDM]behavior %d traj num of states: %d.\n",
         static_cast<int>(ego_behavior), static_cast<int>(traj->size()));
  return kSuccess;
}

ErrorType BehaviorPlanner::EvaluateMultiPolicyTrajs(
    const std::vector<LateralBehavior>& valid_behaviors,
    const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
    const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>&
        valid_surround_trajs,
    LateralBehavior* winner_behavior,
    vec_E<common::Vehicle>* winner_forward_traj, decimal_t* winner_score,
    decimal_t* desired_vel) {
  // 三组候选数据依赖相同下标关联；这里只以行为数组长度作为候选数量。
  int num_valid_behaviors = static_cast<int>(valid_behaviors.size());
  if (num_valid_behaviors < 1) return kWrongStatus;

  // winner 初始为未定义和无穷大代价，随后保留第一个严格更小的候选。
  vec_E<common::Vehicle> traj;
  LateralBehavior behavior = common::LateralBehavior::kUndefined;
  decimal_t min_score = kInf;
  decimal_t des_vel = 0.0;
  for (int i = 0; i < num_valid_behaviors; i++) {
    decimal_t score, vel;
    // 单候选评估同时给出总代价和该自车轨迹对应的建议速度。
    EvaluateSinglePolicyTraj(valid_behaviors[i], valid_forward_trajs[i],
                             valid_surround_trajs[i], &score, &vel);
    // printf("[Stuck]id: %d behavior %d with cost: %lf.\n", ego_id_,
    //        static_cast<int>(valid_behaviors[i]), score);

    if (score < min_score) {
      // 使用严格小于保证同分时保留候选枚举顺序中更靠前的行为。
      min_score = score;
      des_vel = vel;
      behavior = valid_behaviors[i];
      traj = valid_forward_trajs[i];
    }
  }
  printf("[MPDM]choose behavior %d with cost %lf.\n",
         static_cast<int>(behavior), min_score);
  // 将最优候选的四项结果复制到规划主流程的输出缓存。
  *winner_forward_traj = traj;
  *winner_behavior = behavior;
  *winner_score = min_score;
  *desired_vel = des_vel;
  return kSuccess;
}

ErrorType BehaviorPlanner::EvaluateSafetyCost(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost) {
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }
  // 两条轨迹按相同时间下标配对，不进行时间戳插值或重采样。
  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;
  for (int i = 0; i < num_states; i++) {
    // 宽度、长度各增加 1 m，为几何相交检查加入固定空间裕量。
    common::Vehicle inflated_a, inflated_b;
    common::SemanticsUtils::InflateVehicleBySize(traj_a[i], 1.0, 1.0,
                                                 &inflated_a);
    common::SemanticsUtils::InflateVehicleBySize(traj_b[i], 1.0, 1.0,
                                                 &inflated_b);
    bool is_collision = false;
    map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                       inflated_b.param(), inflated_b.state(),
                                       &is_collision);
    if (is_collision) {
      // 每个碰撞采样仅按两车速度差施加软惩罚，并沿时间维累加。
      cost_tmp +=
          0.01 * fabs(traj_a[i].state().velocity - traj_b[i].state().velocity) *
          0.5;
    }
  }
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType BehaviorPlanner::EvaluateSinglePolicyTraj(
    const LateralBehavior& behavior, const vec_E<common::Vehicle>& forward_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
    decimal_t* score, decimal_t* desired_vel) {
  // 只取每辆周车 rollout 的末状态，构造终端时刻的前车查询集合。
  common::VehicleSet vehicle_set;
  for (auto it = surround_traj.begin(); it != surround_traj.end(); ++it) {
    if (!it->second.empty()) {
      vehicle_set.vehicles.insert(std::make_pair(it->first, it->second.back()));
    }
  }

  // 效率第一项：自车终端速度与当前参考期望速度的绝对误差，经固定尺度 10 归一化。
  common::Vehicle ego_vehicle_terminal = forward_traj.back();
  decimal_t cost_efficiency_ego_to_desired_vel =
      fabs(ego_vehicle_terminal.state().velocity -
           reference_desired_velocity_) /
      10.0;
  common::Vehicle leading_vehicle;
  common::Lane ego_ref_lane;
  const decimal_t max_backward_len = 10.0;
  decimal_t forward_lane_len =
      std::max(ego_vehicle_terminal.state().velocity * 10.0, 50.0);
  // 在自车终端状态处重新构造 LK 参考车道，用于搜索终端前车。
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_terminal.state(), p_route_planner_->navi_path(),
          common::LateralBehavior::kLaneKeeping, forward_lane_len,
          max_backward_len, false, &ego_ref_lane) != kSuccess) {
    printf("fail to get ego ref lane duration evaluation for behavior %d.\n",
           static_cast<int>(behavior));
  }

  decimal_t cost_efficiency_leading_to_desired_vel = 0.0;
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = 2.2;
  // 效率第二项仅在终端找到前车，且自车/前车均低于期望速度并相距 100 m 内时启用。
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_ref_lane, ego_vehicle_terminal.state(), vehicle_set, lat_range,
          &leading_vehicle, &distance_residual_ratio) == kSuccess) {
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_vehicle_terminal.state().vec_position)
            .norm();

    if (ego_vehicle_terminal.state().velocity < reference_desired_velocity_ &&
        leading_vehicle.state().velocity < reference_desired_velocity_ &&
        distance_to_leading_vehicle < 100.0) {
      cost_efficiency_leading_to_desired_vel =
          // residual ratio 越接近搜索起点越大，再除以至少 2 m 的欧氏距离。
          1.5 * distance_residual_ratio *
          fabs(ego_vehicle_terminal.state().velocity -
               reference_desired_velocity_) /
          std::max(2.0, distance_to_leading_vehicle);
    }
  }
  // 自车速度误差项和前车阻塞项等权平均。
  decimal_t cost_efficiency = 0.5 * (cost_efficiency_ego_to_desired_vel +
                                     cost_efficiency_leading_to_desired_vel);

  // 安全项：自车轨迹分别与每辆周车轨迹计算软碰撞代价，然后直接求和。
  decimal_t cost_safety = 0.0;
  for (auto& traj : surround_traj) {
    decimal_t safety_tmp = 0.0;
    EvaluateSafetyCost(forward_traj, traj.second, &safety_tmp);
    cost_safety += safety_tmp;
  }

  // 动作项：所有非 LK 行为都施加相同的固定 0.5 换道惩罚。
  decimal_t cost_action = 0.0;
  if (behavior != common::LateralBehavior::kLaneKeeping) {
    cost_action += 0.5;
  }
  // 三类代价没有额外可配置权重，直接相加得到候选总分。
  *score = cost_action + cost_safety + cost_efficiency;
  printf(
      "[CostDebug]behaivor %d: (action %lf, safety %lf, efficiency ego %lf, "
      "leading %lf).\n",
      static_cast<int>(behavior), cost_action, cost_safety,
      cost_efficiency_ego_to_desired_vel,
      cost_efficiency_leading_to_desired_vel);
  // 建议速度独立由整条自车轨迹的曲率/速度扫描结果给出。
  GetDesiredVelocityOfTrajectory(forward_traj, desired_vel);
  return kSuccess;
}

ErrorType BehaviorPlanner::GetDesiredVelocityOfTrajectory(
    const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel) {
  decimal_t min_vel = kInf;
  decimal_t max_acc_normal = 0.0;
  for (auto& v : vehicle_vec) {
    auto state = v.state();
    // 计算离心加速度幅值 |kappa|*v^2；baseline 中 max_acc_normal 始终保持为 0。
    auto acc_normal = fabs(state.curvature) * pow(state.velocity, 2);
    // 因阈值未更新，每个正横向加速度采样都会覆盖结果，最终保留最后一次速度。
    min_vel = acc_normal > max_acc_normal ? state.velocity : min_vel;
  }
  *vel = min_vel;
  return kSuccess;
}

ErrorType BehaviorPlanner::MultiAgentSimForward(
    const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
    vec_E<common::Vehicle>* traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // 初始化输出：自车轨迹与各周车轨迹均包含当前时刻的初始状态。
  traj->clear();
  traj->push_back(semantic_vehicle_set.semantic_vehicles.at(ego_id).vehicle);

  surround_trajs->clear();
  for (const auto &v : semantic_vehicle_set.semantic_vehicles) {
    if (v.first == ego_id) continue;
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
    surround_trajs->at(v.first).push_back(v.second.vehicle);
  }

  // 仿真步数向下取整；临时语义集合保存每个时间切片提交后的全体车辆状态。
  int num_steps_forward = static_cast<int>(sim_horizon_ / sim_resolution_);

  common::SemanticVehicleSet semantic_vehicle_set_tmp = semantic_vehicle_set;

  TicToc timer;
  for (int i = 0; i < num_steps_forward; i++) {
    timer.tic();
    // 本步只写入缓存，待所有车辆计算完成后统一更新，形成同步多智能体 rollout。
    std::unordered_map<int, State> state_cache;
    for (auto& v : semantic_vehicle_set_tmp.semantic_vehicles) {
      // 周车保持初始速度为期望速度，自车使用当前规划周期的参考期望速度。
      decimal_t desired_vel = semantic_vehicle_set.semantic_vehicles.at(v.first)
                                  .vehicle.state()
                                  .velocity;
      decimal_t init_stamp = semantic_vehicle_set.semantic_vehicles.at(v.first)
                                 .vehicle.state()
                                  .time_stamp;
      if (v.first == ego_id) desired_vel = reference_desired_velocity_;

      // 构造排除当前被仿真车辆的环境车辆集合，供前车查询使用。
      common::VehicleSet vehicle_set;
      for (auto& v_other : semantic_vehicle_set_tmp.semantic_vehicles) {
        if (v_other.first != v.first)
          vehicle_set.vehicles.insert(
              std::make_pair(v_other.first, v_other.second.vehicle));
      }

      // 若能获得车道限速，则为 IDM 预留 10% 裕量并限制本车期望速度。
      decimal_t speed_limit;
      if (map_itf_->GetSpeedLimit(v.second.vehicle.state(), v.second.lane,
                                  &speed_limit) == kSuccess) {
        desired_vel = std::min(speed_limit * 0.9, desired_vel);
      }
      sim_param_.idm_param.kDesiredVelocity = desired_vel;

      common::Vehicle leading_vehicle;
      common::State state;
      decimal_t distance_residual_ratio = 0.0;
      const decimal_t lat_range = 2.2;
      // 在固定参考车道上搜索前车；若当前状态已与前车碰撞，则本次 rollout 失败。
      if (map_itf_->GetLeadingVehicleOnLane(
              v.second.lane, v.second.vehicle.state(), vehicle_set, lat_range,
              &leading_vehicle, &distance_residual_ratio) == kSuccess) {
        bool is_collision = false;
        map_itf_->CheckCollisionUsingState(
            v.second.vehicle.param(), v.second.vehicle.state(),
            leading_vehicle.param(), leading_vehicle.state(), &is_collision);
        if (is_collision) {
          return kWrongStatus;
        }
      }
      // 使用前车（若未查询到则为空车辆）执行一次车道坐标系下的跟驰传播。
      if (planning::OnLaneForwardSimulation::PropagateOnce(
              common::StateTransformer(v.second.lane), v.second.vehicle,
              leading_vehicle, sim_resolution_, sim_param_,
              &state) != kSuccess) {
        printf("[MPDM]fail to forward with leading vehicle.\n");
        return kWrongStatus;
      }

      // 统一以初始时间戳和离散步号生成预测时间，并缓存本车下一状态。
      state.time_stamp = init_stamp + (i + 1) * sim_resolution_;
      state_cache.insert(std::make_pair(v.first, state));
    }
    // printf("[Summary]single propogate once time: %lf.\n", timer.toc());

    // 所有车辆传播完成后同步提交状态，并分别追加到自车或对应周车轨迹。
    for (auto& s : state_cache) {
      semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle.set_state(
          s.second);
      if (s.first == ego_id) {
        traj->push_back(
            semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
      } else {
        surround_trajs->at(s.first).push_back(
            semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
      }
    }
  }
  return kSuccess;
}

ErrorType BehaviorPlanner::ConstructReferenceLane(
    const LateralBehavior& lat_behavior, Lane* lane) {
  if (map_itf_ == nullptr) return kWrongStatus;

  int target_lane_id;
  if (lat_behavior == common::LateralBehavior::kLaneKeeping ||
      lat_behavior == common::LateralBehavior::kUndefined) {
    target_lane_id = ego_lane_id_;
    // printf("[BP]Commanding keep lane %d.\n", target_lane_id);
  } else if (lat_behavior == common::LateralBehavior::kLaneChangeLeft) {
    if (map_itf_->GetLeftLaneId(ego_lane_id_, &target_lane_id) != kSuccess) {
      printf("[BP]Commanding a left lane change, but no existing lane.\n");
      target_lane_id = ego_lane_id_;
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
    }
    // printf("[BP]Commanding a left lane change %d --> %d.\n", ego_lane_id_,
    //        target_lane_id);
  } else if (lat_behavior == common::LateralBehavior::kLaneChangeRight) {
    if (map_itf_->GetRightLaneId(ego_lane_id_, &target_lane_id) != kSuccess) {
      printf("[BP]Commanding a right lane change, but no existing lane.\n");
      target_lane_id = ego_lane_id_;
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
    }
    // printf("[BP]Commanding a right lane change %d --> %d.\n", ego_lane_id_,
    //        target_lane_id);
  } else {
    assert(false);
  }

  State ego_state;
  map_itf_->GetEgoState(&ego_state);

  const decimal_t kMaxReflaneDist = 150.0;
  const decimal_t kBackwardDist = 20.0;
  vec_Vecf<2> samples;
  if (map_itf_->GetLocalLaneSamplesByState(
          ego_state, target_lane_id, p_route_planner_->navi_path(),
          kMaxReflaneDist, kBackwardDist, &samples) != kSuccess) {
    return kWrongStatus;
  }

  if (ConstructLaneFromSamples(samples, lane) != kSuccess) {
    return kWrongStatus;
  }

  common::StateTransformer stf(*lane);
  common::FrenetState current_fs;
  stf.GetFrenetStateFromState(ego_state, &current_fs);

  // decimal_t c, cc;
  // decimal_t resolution = 10.0;
  // decimal_t v_max_by_curvature;
  // decimal_t v_ref = kInf;
  // const decimal_t sim_lat_max = 1.5;
  // for (decimal_t s = current_fs.vec_s[0]; s < lane->end(); s += resolution) {
  //   if (lane->GetCurvatureByArcLength(s, &c, &cc) == kSuccess) {
  //     v_max_by_curvature = sqrt(sim_lat_max / fabs(c));
  //     v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
  //   }
  // }
  // reference_desired_velocity_ =
  //     std::min(std::max(v_ref - 3.0, 0.0), user_desired_velocity_);

  decimal_t c, cc;
  decimal_t v_max_by_curvature;
  decimal_t v_ref = kInf;
  const decimal_t sim_lat_max = 1.5;
  decimal_t a_comfort = 1.67;
  decimal_t t_forward = ego_state.velocity / a_comfort;
  decimal_t s_forward =
      std::min(std::max(20.0, t_forward * ego_state.velocity), lane->end());
  decimal_t resolution = 0.2;

  for (decimal_t s = current_fs.vec_s[0]; s < current_fs.vec_s[0] + s_forward;
       s += resolution) {
    if (lane->GetCurvatureByArcLength(s, &c, &cc) == kSuccess) {
      v_max_by_curvature = sqrt(sim_lat_max / fabs(c));
      v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
    }
  }

  reference_desired_velocity_ =
      std::floor(std::min(std::max(v_ref - 2.0, 0.0), user_desired_velocity_));

  // printf("[DEBUG]reference desired vel %lf.\n", reference_desired_velocity_);
  return kSuccess;
}

ErrorType BehaviorPlanner::ConstructLaneFromSamples(
    const vec_E<Vecf<2>>& samples, Lane* lane) {
  double d = 0.0;
  std::vector<decimal_t> para;
  para.push_back(d);

  int num_samples = static_cast<int>(samples.size());
  for (int i = 1; i < num_samples; i++) {
    double dx = samples[i](0) - samples[i - 1](0);
    double dy = samples[i](1) - samples[i - 1](1);
    d += std::hypot(dx, dy);
    para.push_back(d);
  }

  const int num_segments = 20;
  Eigen::ArrayXf breaks =
      Eigen::ArrayXf::LinSpaced(num_segments, para.front(), para.back());

  const decimal_t regulator = (double)1e6;
  if (common::LaneGenerator::GetLaneBySampleFitting(
          samples, para, breaks, regulator, lane) != kSuccess) {
    // printf("fitting error.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType BehaviorPlanner::JudgeBehaviorByLaneId(
    const int ego_lane_id_by_pos, LateralBehavior* behavior_by_lane_id) {
  if (ego_lane_id_by_pos == ego_lane_id_) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  auto it = std::find(potential_lk_lane_ids_.begin(),
                      potential_lk_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcl = std::find(potential_lcl_lane_ids_.begin(),
                          potential_lcl_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcr = std::find(potential_lcr_lane_ids_.begin(),
                          potential_lcr_lane_ids_.end(), ego_lane_id_by_pos);

  if (it != potential_lk_lane_ids_.end()) {
    // ~ if routing information is available, here
    // ~ we still need to check whether the change is consist with the
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  if (it_lcl != potential_lcl_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeLeft;
    return kSuccess;
  }

  if (it_lcr != potential_lcr_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeRight;
    return kSuccess;
  }

  *behavior_by_lane_id = common::LateralBehavior::kUndefined;
  return kSuccess;
}

ErrorType BehaviorPlanner::UpdateEgoBehavior(
    const LateralBehavior& behavior_by_lane_id) {
  if (behavior_.lat_behavior == common::LateralBehavior::kLaneKeeping) {
    if (behavior_by_lane_id == common::LateralBehavior::kLaneKeeping) {
      // ~ lane keeping
    } else if (behavior_by_lane_id == common::LateralBehavior::kUndefined) {
      // ~ observed lane change is jumping without logic, keep current
      // ~ behavior
    } else {
      // ~ observed wrong behavior, cause undefined system behavior
      behavior_.lat_behavior = common::LateralBehavior::kUndefined;
    }
  } else if (behavior_.lat_behavior ==
             common::LateralBehavior::kLaneChangeLeft) {
    if (behavior_by_lane_id == common::LateralBehavior::kLaneKeeping) {
      // ~ still in the course of lane changing
    } else if (behavior_by_lane_id ==
               common::LateralBehavior::kLaneChangeLeft) {
      // ~ lane change complete
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else if (behavior_by_lane_id == common::LateralBehavior::kUndefined) {
      // ~ lane id jumping in lane change, cancel lane change
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else {
      // ~ wrong behavior
      behavior_.lat_behavior = common::LateralBehavior::kUndefined;
      lock_to_hmi_ = false;
    }
  } else if (behavior_.lat_behavior ==
             common::LateralBehavior::kLaneChangeRight) {
    if (behavior_by_lane_id == common::LateralBehavior::kLaneKeeping) {
      // ~ still in the course of lane changing
    } else if (behavior_by_lane_id ==
               common::LateralBehavior::kLaneChangeRight) {
      // ~ lane change complete
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else if (behavior_by_lane_id == common::LateralBehavior::kUndefined) {
      // ~ lane id jumping in lane change, cancel lane change
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else {
      behavior_.lat_behavior = common::LateralBehavior::kUndefined;
      lock_to_hmi_ = false;
    }
  }  // end enumerating current behaviors
  return kSuccess;
}

ErrorType BehaviorPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;
  GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
                      &potential_lk_lane_ids_);
  GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeLeft,
                      &potential_lcl_lane_ids_);
  GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeRight,
                      &potential_lcr_lane_ids_);
  return kSuccess;
}

ErrorType BehaviorPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) {
  candidate_lane_ids->clear();
  if (beh == common::LateralBehavior::kUndefined ||
      beh == common::LateralBehavior::kLaneKeeping) {
    map_itf_->GetChildLaneIds(source_lane_id, candidate_lane_ids);
  } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
    int l_lane_id;
    if (map_itf_->GetLeftLaneId(source_lane_id, &l_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(l_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(l_lane_id);
    }
  } else if (beh == common::LateralBehavior::kLaneChangeRight) {
    int r_lane_id;
    if (map_itf_->GetRightLaneId(source_lane_id, &r_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(r_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(r_lane_id);
    }
  } else {
    assert(false);
  }
  return kSuccess;
}

void BehaviorPlanner::set_map_interface(BehaviorPlannerMapItf* itf) {
  map_itf_ = itf;
}

void BehaviorPlanner::set_hmi_behavior(const LateralBehavior& hmi_behavior) {
  // ~ hmi interface is enabled when >= L2
  if (autonomous_level_ == 2) {
    behavior_.lat_behavior = hmi_behavior;
    hmi_behavior_ = hmi_behavior;
    lock_to_hmi_ = true;
  } else if (autonomous_level_ == 3) {
    hmi_behavior_ = hmi_behavior;
    lock_to_hmi_ = true;
  }
}

void BehaviorPlanner::set_autonomous_level(int level) {
  autonomous_level_ = level;
}

void BehaviorPlanner::set_aggressive_level(int level) {
  aggressive_level_ = level;
}

void BehaviorPlanner::set_user_desired_velocity(const decimal_t desired_vel) {
  if (autonomous_level_ >= 2) {
    user_desired_velocity_ = std::max(desired_vel, 0.0);
  }
}

void BehaviorPlanner::set_use_sim_state(bool use_sim_state) {
  use_sim_state_ = use_sim_state;
}

void BehaviorPlanner::set_sim_resolution(const decimal_t sim_resolution) {
  sim_resolution_ = sim_resolution;
}

void BehaviorPlanner::set_sim_horizon(const decimal_t sim_horizon) {
  sim_horizon_ = sim_horizon;
}

decimal_t BehaviorPlanner::user_desired_velocity() const {
  return user_desired_velocity_;
}

decimal_t BehaviorPlanner::reference_desired_velocity() const {
  return reference_desired_velocity_;
}

BehaviorPlanner::Behavior BehaviorPlanner::behavior() const {
  return behavior_;
}

vec_E<vec_E<common::Vehicle>> BehaviorPlanner::forward_trajs() const {
  return forward_trajs_;
}

std::vector<BehaviorPlanner::LateralBehavior>
BehaviorPlanner::forward_behaviors() const {
  return forward_behaviors_;
}

int BehaviorPlanner::autonomous_level() const { return autonomous_level_; }

}  // namespace planning
