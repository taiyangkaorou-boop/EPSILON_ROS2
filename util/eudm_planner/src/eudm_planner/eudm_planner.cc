/**
 * @file eudm_planner.cc
 * @brief EUDM 配置、候选编排、交互前向仿真和代价选择实现。
 */

#include "eudm_planner/eudm_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "common/rss/rss_checker.h"

namespace planning {

// 返回用于日志和通用 Planner 识别的名称。
std::string EudmPlanner::Name() { return std::string("Eudm behavior planner"); }

// 从 protobuf text 文件解析完整 EUDM 配置。
ErrorType EudmPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading eudm planner config\n");
  using namespace google::protobuf;
  // 直接用 POSIX fd 构造 protobuf 输入流；当前未检查 open/Parse 返回值。
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  TextFormat::Parse(&fstream, &cfg_);
  // 只在 required 字段不完整时 assert；release 构建可能继续返回成功。
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

// 把 protobuf 中的 IDM、jerk、纯跟踪和横向约束逐项映射到传播器参数。
ErrorType EudmPlanner::GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                                   OnLaneForwardSimulation::Param* sim_param) {
  sim_param->idm_param.kMinimumSpacing = cfg.lon().idm().min_spacing();
  sim_param->idm_param.kDesiredHeadwayTime = cfg.lon().idm().head_time();
  sim_param->idm_param.kAcceleration = cfg.lon().limit().acc();
  sim_param->idm_param.kComfortableBrakingDeceleration =
      cfg.lon().limit().soft_brake();
  sim_param->idm_param.kHardBrakingDeceleration =
      cfg.lon().limit().hard_brake();
  sim_param->idm_param.kExponent = cfg.lon().idm().exponent();
  sim_param->max_lon_acc_jerk = cfg.lon().limit().acc_jerk();
  sim_param->max_lon_brake_jerk = cfg.lon().limit().brake_jerk();
  sim_param->steer_control_gain = cfg.lat().pure_pursuit().gain();
  sim_param->steer_control_max_lookahead_dist =
      cfg.lat().pure_pursuit().max_lookahead_dist();
  sim_param->steer_control_min_lookahead_dist =
      cfg.lat().pure_pursuit().min_lookahead_dist();
  sim_param->max_lat_acceleration_abs = cfg.lat().limit().acc();
  sim_param->max_lat_jerk_abs = cfg.lat().limit().jerk();
  sim_param->max_curvature_abs = cfg.lat().limit().curvature();
  sim_param->max_steer_angle_abs = cfg.lat().limit().steer_angle();
  sim_param->max_steer_rate = cfg.lat().limit().steer_rate();
  sim_param->auto_decelerate_if_lat_failed = cfg.auto_dec_if_lat_failed();
  return kSuccess;
}

// 初始化动作树、两类车辆传播参数和常规/严格 RSS 参数。
ErrorType EudmPlanner::Init(const std::string config) {
  // 当前忽略 ReadConfig 返回值，并在每次 Init 时重新分配裸指针。
  ReadConfig(config);

  dcp_tree_ptr_ = new DcpTree(cfg_.sim().duration().tree_height(),
                              cfg_.sim().duration().layer(),
                              cfg_.sim().duration().last_layer());
  LOG(INFO) << "[Eudm]Init.";
  LOG(INFO) << "[Eudm]ActionScript size: "
            << dcp_tree_ptr_->action_script().size() << std::endl;

  // 自车和周车使用两套独立的前向仿真配置。
  GetSimParam(cfg_.sim().ego(), &ego_sim_param_);
  GetSimParam(cfg_.sim().agent(), &agent_sim_param_);

  // 常规 RSS 用于软风险代价。
  rss_config_ = common::RssChecker::RssConfig(
      cfg_.safety().rss().response_time(),
      cfg_.safety().rss().longitudinal_acc_max(),
      cfg_.safety().rss().longitudinal_brake_min(),
      cfg_.safety().rss().longitudinal_brake_max(),
      cfg_.safety().rss().lateral_acc_max(),
      cfg_.safety().rss().lateral_brake_min(),
      cfg_.safety().rss().lateral_brake_max(),
      cfg_.safety().rss().lateral_miu());

  // 自车作为前车时，用更严格参数约束后方来车。
  rss_config_strict_as_front_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_front().response_time(),
      cfg_.safety().rss_strict_as_front().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_acc_max(),
      cfg_.safety().rss_strict_as_front().lateral_brake_min(),
      cfg_.safety().rss_strict_as_front().lateral_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_miu());

  // 自车作为后车时，用更严格参数约束前方车辆间距。
  rss_config_strict_as_rear_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_rear().response_time(),
      cfg_.safety().rss_strict_as_rear().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_acc_max(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_min(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_miu());

  return kSuccess;
}

// 把 DCP 的三类横向/纵向动作转换为 common 行为枚举。
ErrorType EudmPlanner::TranslateDcpActionToLonLatBehavior(
    const DcpAction& action, LateralBehavior* lat,
    LongitudinalBehavior* lon) const {
  switch (action.lat) {
    case DcpLatAction::kLaneKeeping: {
      *lat = LateralBehavior::kLaneKeeping;
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      *lat = LateralBehavior::kLaneChangeLeft;
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      *lat = LateralBehavior::kLaneChangeRight;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Lateral action translation error!";
      return kWrongStatus;
    }
  }

  switch (action.lon) {
    case DcpLonAction::kMaintain: {
      *lon = LongitudinalBehavior::kMaintain;
      break;
    }
    case DcpLonAction::kAccelerate: {
      *lon = LongitudinalBehavior::kAccelerate;
      break;
    }
    case DcpLonAction::kDecelerate: {
      *lon = LongitudinalBehavior::kDecelerate;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Longitudinal action translation error!";
      return kWrongStatus;
    }
  }

  return kSuccess;
}

// 提取序列首次换道时刻、长期横向行为，并识别换道后回到 LK 的取消序列。
ErrorType EudmPlanner::ClassifyActionSeq(
    const std::vector<DcpAction>& action_seq, decimal_t* operation_at_seconds,
    common::LateralBehavior* lat_behavior, bool* is_cancel_operation) const {
  decimal_t duration = 0.0;
  decimal_t operation_at = 0.0;
  bool find_lat_active_behavior = false;
  *is_cancel_operation = false;
  // 在第一个 LCL/LCR 出现前累计动作时长。
  for (const auto& action : action_seq) {
    if (!find_lat_active_behavior) {
      if (action.lat == DcpLatAction::kLaneChangeLeft) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeLeft;
        find_lat_active_behavior = true;
      }
      if (action.lat == DcpLatAction::kLaneChangeRight) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeRight;
        find_lat_active_behavior = true;
      }
    } else {
      if (action.lat == DcpLatAction::kLaneKeeping) {
        *is_cancel_operation = true;
      }
    }
    duration += action.t;
  }
  // 全程 LK 时把“操作时刻”放到规划时域之后一个普通 layer。
  if (!find_lat_active_behavior) {
    *operation_at_seconds = duration + cfg_.sim().duration().layer();
    *lat_behavior = common::LateralBehavior::kLaneKeeping;
    *is_cancel_operation = false;
  }
  return kSuccess;
}

// 为每条候选预分配独立结果槽位，供候选线程按 seq_id 写回。
ErrorType EudmPlanner::PrepareMultiThreadContainers(const int n_sequence) {
  LOG(INFO) << "[Eudm][Process]Prepare multi-threading - " << n_sequence
            << " threads.";

  sim_res_.clear();
  sim_res_.resize(n_sequence, 0);

  risky_res_.clear();
  risky_res_.resize(n_sequence, 0);

  sim_info_.clear();
  sim_info_.resize(n_sequence, std::string(""));

  final_cost_.clear();
  final_cost_.resize(n_sequence, 0.0);

  progress_cost_.clear();
  progress_cost_.resize(n_sequence);

  tail_cost_.clear();
  tail_cost_.resize(n_sequence);

  forward_trajs_.clear();
  forward_trajs_.resize(n_sequence);

  forward_lat_behaviors_.clear();
  forward_lat_behaviors_.resize(n_sequence);

  forward_lon_behaviors_.clear();
  forward_lon_behaviors_.resize(n_sequence);

  surround_trajs_.clear();
  surround_trajs_.resize(n_sequence);
  return kSuccess;
}

// 把语义周车快照转换为固定 Lane 上传播的仿真 agent。
ErrorType EudmPlanner::GetSurroundingForwardSimAgents(
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    ForwardSimAgentSet* fsagents) const {
  for (const auto& psv : surrounding_semantic_vehicles.semantic_vehicles) {
    ForwardSimAgent fsagent;

    int id = psv.second.vehicle.id();
    fsagent.id = id;
    fsagent.vehicle = psv.second.vehicle;

    // 周车继承统一传播参数，再按当前加速度估计本时域期望速度。
    fsagent.sim_param = agent_sim_param_;

    common::State state = psv.second.vehicle.state();
    // 非负加速度按当前速度匀速；负加速度外推到规划时域末端且不低于零。
    if (state.acceleration >= 0) {
      fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
    } else {
      decimal_t est_vel =
          std::max(0.0, state.velocity + state.acceleration * sim_time_total_);
      fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
    }

    // 保存语义预测，但后续传播使用当前 lane/stf，不在此处重采样行为。
    fsagent.lat_probs = psv.second.probs_lat_behaviors;
    fsagent.lat_behavior = psv.second.lat_behavior;

    fsagent.lane = psv.second.lane;
    fsagent.stf = common::StateTransformer(fsagent.lane);

    // 横向命中范围控制前车搜索的 Lane 邻域宽度。
    fsagent.lat_range = cfg_.sim().agent().cooperative_lat_range();

    fsagents->forward_sim_agents.insert(std::make_pair(id, fsagent));
  }

  return kSuccess;
}

// 获取关键周车，为每条 DCP 候选创建线程，汇总仿真状态并选择最低代价序列。
ErrorType EudmPlanner::RunEudm() {
  // 从语义地图读取带预测行为和参考 Lane 的关键周车。
  common::SemanticVehicleSet surrounding_semantic_vehicles;
  if (map_itf_->GetKeySemanticVehicles(&surrounding_semantic_vehicles) !=
      kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get key semantic vehicles. Exit";
    return kWrongStatus;
  }

  ForwardSimAgentSet surrounding_fsagents;
  GetSurroundingForwardSimAgents(surrounding_semantic_vehicles,
                                 &surrounding_fsagents);

  auto action_script = dcp_tree_ptr_->action_script();
  int n_sequence = action_script.size();

  // 线程数与候选数相同，结果容器在启动线程前固定尺寸。
  std::vector<std::thread> thread_set(n_sequence);
  PrepareMultiThreadContainers(n_sequence);

  // 每条候选独立复制输入并执行前向仿真；当前没有线程池或并发上限。
  // TODO(@lu.zhang) 使用线程池复用工作线程。
  TicToc timer;
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i] =
        std::thread(&EudmPlanner::SimulateActionSequence, this, ego_vehicle_,
                    surrounding_fsagents, action_script[i], i);
  }
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i].join();
  }

  LOG(INFO) << "[Eudm][Process]Multi-thread forward simulation finished!";

  // 汇总成功候选；只要至少一条成功就进入代价选择。
  bool sim_success = false;
  int num_valid_behaviors = 0;
  for (int i = 0; i < static_cast<int>(sim_res_.size()); ++i) {
    if (sim_res_[i] == 1) {
      sim_success = true;
      num_valid_behaviors++;
    }
  }

  // 输出每条脚本的 M/A/D、K/L/R、有效性、风险、总代价和逐层分项。
  for (int i = 0; i < n_sequence; ++i) {
    std::ostringstream line_info;
    line_info << "[Eudm][Result]" << i << " [";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << final_cost_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    if (sim_res_[i]) {
      line_info << "[Eudm][Result][e;s;n;w:";
      for (const auto& c : progress_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << c.efficiency.ego_to_desired_vel << "_"
                  << c.efficiency.leading_to_desired_vel << ";" << c.safety.rss
                  << "_" << c.safety.occu_lane << ";"
                  << c.navigation.lane_change_preference << ";" << c.weight;
        line_info << "|";
      }
      line_info << "]";
    }
    LOG(WARNING) << line_info.str();
  }
  LOG(WARNING) << "[Eudm][Result]Sim status: " << sim_success << " with "
               << num_valid_behaviors << " behaviors.";
  if (!sim_success) {
    LOG(ERROR) << "[Eudm][Fatal]Fail to find any valid behavior. Exit";
    return kWrongStatus;
  }

  // 在仿真成功的候选中选择累计代价最小者。
  if (EvaluateMultiThreadSimResults(&winner_id_, &winner_score_) != kSuccess) {
    LOG(ERROR)
        << "[Eudm][Fatal]fail to evaluate multi-thread sim results. Exit";
    return kWrongStatus;
  }
  return kSuccess;
}

// 将单个 DCP 动作翻译后写入自车仿真上下文。
ErrorType EudmPlanner::UpdateEgoBehaviorsUsingAction(
    const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const {
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    printf("[Eudm]Translate action error\n");
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;
  return kSuccess;
}

// 按整条脚本初始化横向时序模式和纵向 IDM 目标。
ErrorType EudmPlanner::UpdateSimSetupForScenario(
    const std::vector<DcpAction>& action_seq,
    ForwardSimEgoAgent* ego_fsagent) const {
  // 提取首次横向操作时刻、长期方向和是否取消。
  common::LateralBehavior seq_lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  ClassifyActionSeq(action_seq, &operation_at_seconds, &seq_lat_behavior,
                    &is_cancel_behavior);
  ego_fsagent->operation_at_seconds = operation_at_seconds;
  ego_fsagent->is_cancel_behavior = is_cancel_behavior;
  ego_fsagent->seq_lat_behavior = seq_lat_behavior;

  // 将脚本归为始终 LK、延迟换道、立即换道或换道后取消。
  if (is_cancel_behavior) {
    ego_fsagent->lat_behavior_longterm = LateralBehavior::kLaneKeeping;
    ego_fsagent->seq_lat_mode = LatSimMode::kChangeThenCancel;
  } else {
    if (seq_lat_behavior == LateralBehavior::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneKeep;
    } else if (action_seq.front().lat == DcpLatAction::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kKeepThenChange;
    } else {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneChange;
    }
    ego_fsagent->lat_behavior_longterm = seq_lat_behavior;
  }

  // 以当前速度下取整为基准，根据纵向动作调整 IDM 期望速度。
  decimal_t desired_vel = std::floor(ego_fsagent->vehicle.state().velocity);
  simulator::IntelligentDriverModel::Param idm_param_tmp;
  idm_param_tmp = ego_sim_param_.idm_param;

  // 当前直接读取第 2 层动作；依赖脚本长度至少为 2。
  switch (action_seq[1].lon) {
    case DcpLonAction::kAccelerate: {
      idm_param_tmp.kDesiredVelocity = std::min(
          desired_vel + cfg_.sim().acc_cmd_vel_gap(), desired_velocity_);
      // 加速动作同时缩短 IDM 最小间距和期望时距，使行为更激进。
      idm_param_tmp.kMinimumSpacing *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      idm_param_tmp.kDesiredHeadwayTime *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      break;
    }
    case DcpLonAction::kDecelerate: {
      idm_param_tmp.kDesiredVelocity =
          std::min(std::max(desired_vel - cfg_.sim().dec_cmd_vel_gap(), 0.0),
                   desired_velocity_);
      break;
    }
    case DcpLonAction::kMaintain: {
      idm_param_tmp.kDesiredVelocity = std::min(desired_vel, desired_velocity_);
      break;
    }
    default: {
      printf("[Eudm]Error - Lon action not valid\n");
      assert(false);
    }
  }
  ego_fsagent->sim_param = ego_sim_param_;
  ego_fsagent->sim_param.idm_param = idm_param_tmp;
  ego_fsagent->lat_range = cfg_.sim().ego().cooperative_lat_range();

  return kSuccess;
}

// 为一个动作层更新离散行为、参考 Lane、目标 gap 和换道前置 RSS 门限。
ErrorType EudmPlanner::UpdateSimSetupForLayer(
    const DcpAction& action, const ForwardSimAgentSet& other_fsagent,
    ForwardSimEgoAgent* ego_fsagent) const {
  // DCP 动作转换为本层 common 纵横向行为。
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;

  // 参考 Lane 前向长度随车速线性增长，并截断到配置上下界。
  auto state = ego_fsagent->vehicle.state();
  decimal_t forward_lane_len =
      std::min(std::max(state.velocity * cfg_.sim().ref_line().len_vel_coeff(),
                        cfg_.sim().ref_line().forward_len_min()),
               cfg_.sim().ref_line().forward_len_max());

  // current lane 始终按 LK 构造，表示换道前所在车道。
  common::Lane lane_current;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), LateralBehavior::kLaneKeeping,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_current) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->current_lane = lane_current;
  ego_fsagent->current_stf = common::StateTransformer(lane_current);

  // target lane 按本层横向行为构造，供当前层跟踪和 gap 搜索。
  common::Lane lane_target;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_target) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->target_lane = lane_target;
  ego_fsagent->target_stf = common::StateTransformer(lane_target);

  // longterm lane 按整条序列的最终横向行为构造。
  common::Lane lane_longterm;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior_longterm,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_longterm) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->longterm_lane = lane_longterm;
  ego_fsagent->longterm_stf = common::StateTransformer(lane_longterm);

  // 换道层在目标 Lane 上锁定当前前/后车 ID，层内不重新选择 gap。
  if (ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping) {
    common::VehicleSet other_vehicles;
    for (const auto& pv : other_fsagent.forward_sim_agents) {
      other_vehicles.vehicles.insert(
          std::make_pair(pv.first, pv.second.vehicle));
    }

    bool has_front_vehicle = false, has_rear_vehicle = false;
    common::Vehicle front_vehicle, rear_vehicle;
    common::FrenetState front_fs, rear_fs;
    // 当前忽略查询返回值，失败时依赖 has_* 的初始 false。
    map_itf_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ego_fsagent->target_lane, state, other_vehicles, &has_front_vehicle,
        &front_vehicle, &front_fs, &has_rear_vehicle, &rear_vehicle, &rear_fs);

    ego_fsagent->target_gap_ids(0) =
        has_front_vehicle ? front_vehicle.id() : -1;
    ego_fsagent->target_gap_ids(1) = has_rear_vehicle ? rear_vehicle.id() : -1;

    if (cfg_.safety().rss_for_layers_enable()) {
      // 在进入逐步仿真前，用目标 Lane 前后车执行严格纵向 RSS 门限。
      common::FrenetState ego_fs;
      if (kSuccess != ego_fsagent->target_stf.GetFrenetStateFromState(
                          ego_fsagent->vehicle.state(), &ego_fs)) {
        return kWrongStatus;
      }
      decimal_t s_ego_fbumper = ego_fs.vec_s[0] +
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();
      decimal_t s_ego_rbumper = ego_fs.vec_s[0] -
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();

      if (has_front_vehicle) {
        decimal_t s_front_rbumper = front_fs.vec_s[0] -
                                    front_vehicle.param().length() / 2.0 +
                                    front_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            front_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Front,
            rss_config_strict_as_rear_, &rss_dist);

        // 自车作为后车，前向净距不足即淘汰该动作层。
        if (s_front_rbumper - s_ego_fbumper < rss_dist) {
          return kWrongStatus;
        }
      }

      if (has_rear_vehicle) {
        decimal_t s_rear_fbumper = rear_fs.vec_s[0] +
                                   rear_vehicle.param().length() / 2.0 +
                                   rear_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            rear_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Rear,
            rss_config_strict_as_front_, &rss_dist);

        // 自车作为前车，后向净距不足即淘汰该动作层。
        if (s_ego_rbumper - s_rear_fbumper < rss_dist) {
          return kWrongStatus;
        }
      }
    }
  }

  return kSuccess;
}

// 在一个子场景内逐层传播整条动作脚本，并累计轨迹、行为、代价和风险。
ErrorType EudmPlanner::SimulateScenario(
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
        sub_surround_trajs) {
  // 多层自车轨迹包含规划起点，后续每层追加积分结果。
  vec_E<common::Vehicle> ego_traj_multilayers{ego_vehicle};

  // 每个周车轨迹同样以当前观测状态作为首样本。
  std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multilayers;
  for (const auto& p_fsa : surrounding_fsagents.forward_sim_agents) {
    surround_trajs_multilayers.insert(std::pair<int, vec_E<common::Vehicle>>(
        p_fsa.first, vec_E<common::Vehicle>({p_fsa.second.vehicle})));
  }

  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructure> cost_multilayers;
  std::set<int> risky_ids_multilayers;

  // 场景级上下文在第一层前初始化，此后车辆状态逐层滚动。
  ForwardSimEgoAgent ego_fsagent_this_layer;
  ego_fsagent_this_layer.vehicle = ego_vehicle;
  UpdateSimSetupForScenario(action_seq, &ego_fsagent_this_layer);

  ForwardSimAgentSet surrounding_fsagents_this_layer = surrounding_fsagents;

  int action_ref_lane_id = ego_lane_id_;
  bool is_sub_seq_risky = false;
  // 使用局部脚本副本，允许换道完成后动态重写剩余横向动作。
  std::vector<DcpAction> action_seq_sim = action_seq;

  // 逐个动作层执行参考 Lane/gap 更新、积分、安全检查和代价计算。
  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    auto action_this_layer = action_seq_sim[i];

    // 每层开始时固定 Lane、STF、gap 和行为，层内积分步复用这些上下文。
    if (kSuccess != UpdateSimSetupForLayer(action_this_layer,
                                           surrounding_fsagents_this_layer,
                                           &ego_fsagent_this_layer)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      return kWrongStatus;
    }

    // TODO(lu.zhang): 在此加入 MOBIL/RSS 初筛，提前删除明显不可行换道。
    // if (ego_lat_behavior_this_layer != LateralBehavior::kLaneKeeping) {
    //   if (!CheckLaneChangingFeasibilityUsingMobil(
    //           ego_semantic_vehicle_this_layer,
    //           semantic_vehicle_set_this_layer)) {
    //     return kWrongStatus;
    //   }
    // }

    // 同步传播本层全部积分步，输出不包含本层初始状态。
    vec_E<common::Vehicle> ego_traj_multisteps;
    std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multisteps;
    if (SimulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                             surrounding_fsagents_this_layer,
                             &ego_traj_multisteps,
                             &surround_trajs_multisteps) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      return kWrongStatus;
    }

    // 将本层末状态作为下一层初始状态；零步轨迹会在 back() 处失效。
    ego_fsagent_this_layer.vehicle.set_state(
        ego_traj_multisteps.back().state());
    for (auto it = surrounding_fsagents_this_layer.forward_sim_agents.begin();
         it != surrounding_fsagents_this_layer.forward_sim_agents.end(); ++it) {
      it->second.vehicle.set_state(
          surround_trajs_multisteps.at(it->first).back().state());
    }

    // 对本层全部同步样本执行膨胀车身严格碰撞检查。
    bool is_strictly_safe = false;
    int collided_id = 0;
    TicToc timer;
    if (StrictSafetyCheck(ego_traj_multisteps, surround_trajs_multisteps,
                          &is_strictly_safe, &collided_id) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Check F)");
      return kWrongStatus;
    }
    // LOG(INFO) << "[RssTime]safety check time per action: " << timer.toc();

    if (!is_strictly_safe) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Strict F:") +
                                     std::to_string(collided_id) +
                                     std::string(")");
      return kWrongStatus;
    }

    // Lane ID 进入目标候选集合时，改写剩余脚本以反映提前完成/取消语义。
    int current_lane_id;
    if (CheckIfLateralActionFinished(
            ego_fsagent_this_layer.vehicle.state(), action_ref_lane_id,
            ego_fsagent_this_layer.lat_behavior, &current_lane_id)) {
      action_ref_lane_id = current_lane_id;
      if (kSuccess != UpdateLateralActionSequence(i, &action_seq_sim)) {
        (*sub_sim_res)[sub_seq_id] = 0;
        (*sub_sim_info)[sub_seq_id] += std::string("(Update Lat F)");
        return kWrongStatus;
      }
      ego_fsagent_this_layer.lat_behavior_longterm =
          LateralBehavior::kLaneKeeping;
    }

    // 追加本层自车/周车轨迹和实际采用的离散行为。
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.push_back(ego_fsagent_this_layer.lat_behavior);
    ego_lon_behavior_multilayers.push_back(ego_fsagent_this_layer.lon_behavior);

    for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
      int id = v.first;
      surround_trajs_multilayers.at(id).insert(
          surround_trajs_multilayers.at(id).end(),
          surround_trajs_multisteps.at(id).begin(),
          surround_trajs_multisteps.at(id).end());
    }

    // 计算层代价，收集 RSS 风险 ID，并施加按层指数折扣。
    CostStructure cost;
    bool verbose = false;
    std::set<int> risky_ids;
    bool is_risky_action = false;
    CostFunction(action_this_layer, ego_fsagent_this_layer,
                 surrounding_fsagents_this_layer, ego_traj_multisteps,
                 surround_trajs_multisteps, verbose, &cost, &is_risky_action,
                 &risky_ids);
    if (is_risky_action) {
      is_sub_seq_risky = true;
      for (const auto& id : risky_ids) {
        risky_ids_multilayers.insert(id);
      }
    }
    cost.weight = cost.weight * pow(cfg_.cost().discount_factor(), i);
    cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_multilayers.push_back(cost);
  }

  // 全部动作层成功后，把局部结果写入当前子场景槽位。
  (*sub_sim_res)[sub_seq_id] = 1;
  (*sub_risky_res)[sub_seq_id] = is_sub_seq_risky ? 1 : 0;
  if (is_sub_seq_risky) {
    std::string risky_id_list;
    for (auto it = risky_ids_multilayers.begin();
         it != risky_ids_multilayers.end(); it++) {
      risky_id_list += " " + std::to_string(*it);
    }
    (*sub_sim_info)[sub_seq_id] += std::string("(Risky)") + risky_id_list;
  }
  (*sub_progress_cost)[sub_seq_id] = cost_multilayers;
  (*sub_forward_trajs)[sub_seq_id] = ego_traj_multilayers;
  (*sub_forward_lat_behaviors)[sub_seq_id] = ego_lat_behavior_multilayers;
  (*sub_forward_lon_behaviors)[sub_seq_id] = ego_lon_behavior_multilayers;
  (*sub_surround_trajs)[sub_seq_id] = surround_trajs_multilayers;

  return kSuccess;
}

// 候选线程入口：执行默认子场景，并把子场景 0 的结果写回候选级容器。
ErrorType EudmPlanner::SimulateActionSequence(
    const common::Vehicle& ego_vehicle,
    const ForwardSimAgentSet& surrounding_fsagents,
    const std::vector<DcpAction>& action_seq, const int& seq_id) {
  if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = std::string("(Pre-deleted)");
    return kWrongStatus;
  }

  // 预留多情景分支接口，但当前每条动作序列固定只有一个确定性场景。
  // TODO(@lu.zhang) 在此加入初步安全评估与多模态场景分支。
  int n_sub_threads = 1;

  // 子场景结果使用局部容器，避免不同候选线程写同一内部对象。
  std::vector<int> sub_sim_res(n_sub_threads);
  std::vector<int> sub_risky_res(n_sub_threads);
  std::vector<std::string> sub_sim_info(n_sub_threads);
  std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
  std::vector<CostStructure> sub_tail_cost(n_sub_threads);
  vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
      n_sub_threads);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
      n_sub_threads);
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(
      n_sub_threads);

  SimulateScenario(ego_vehicle, surrounding_fsagents, action_seq, seq_id, 0,
                   &sub_sim_res, &sub_risky_res, &sub_sim_info,
                   &sub_progress_cost, &sub_tail_cost, &sub_forward_trajs,
                   &sub_forward_lat_behaviors, &sub_forward_lon_behaviors,
                   &sub_surround_trajs);

  if (sub_sim_res.front() == 0) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = sub_sim_info.front();
    return kWrongStatus;
  }

  // 当前只选择默认场景 front()，没有概率加权或 worst-case 聚合。
  sim_res_[seq_id] = 1;
  risky_res_[seq_id] = sub_risky_res.front();
  sim_info_[seq_id] = sub_sim_info.front();
  progress_cost_[seq_id] = sub_progress_cost.front();
  tail_cost_[seq_id] = sub_tail_cost.front();
  forward_trajs_[seq_id] = sub_forward_trajs.front();
  forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors.front();
  forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors.front();
  surround_trajs_[seq_id] = sub_surround_trajs.front();

  return kSuccess;
}

// 换道在层末提前完成时，重写后续动作以维持“完成”或“取消回切”的语义。
ErrorType EudmPlanner::UpdateLateralActionSequence(
    const int cur_idx, std::vector<DcpAction>* action_seq) const {
  if (cur_idx == static_cast<int>(action_seq->size()) - 1) {
    return kSuccess;
  }

  switch ((*action_seq)[cur_idx].lat) {
    case DcpLatAction::kLaneKeeping: {
      // LK 完成不改变后续脚本。
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // 左换道已完成：后续 LCL 改为 LK。
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // 原脚本要求取消：完成左换后，后续 LK 改为右换回原 Lane。
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeRight;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // 已存在显式反向动作时拒绝二次改写。
          return kWrongStatus;
        }
      }
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // 右换道已完成：后续 LCR 改为 LK。
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // 原脚本要求取消：完成右换后，后续 LK 改为左换回原 Lane。
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeLeft;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // 已存在显式反向动作时拒绝二次改写。
          return kWrongStatus;
        }
      }
      break;
    }

    default: {
      std::cout << "[Eudm]Error - Invalid lateral behavior" << std::endl;
      assert(false);
    }
  }

  return kSuccess;
}

// 以当前位置最近 Lane 是否落入目标方向候选集判断换道是否完成。
bool EudmPlanner::CheckIfLateralActionFinished(
    const common::State& cur_state, const int& action_ref_lane_id,
    const LateralBehavior& lat_behavior, int* current_lane_id) const {
  if (lat_behavior == LateralBehavior::kLaneKeeping) {
    return false;
  }

  decimal_t current_lane_dist;
  decimal_t arc_len;
  // 当前忽略地图查询返回值，失败时 current_lane_id 可能没有有效输出。
  map_itf_->GetNearestLaneIdUsingState(cur_state.ToXYTheta(),
                                       std::vector<int>(), current_lane_id,
                                       &current_lane_dist, &arc_len);
  // std::cout << "[Eudm] current_lane_id: " << *current_lane_id << std::endl;
  // std::cout << "[Eudm] action_ref_lane_id: " << action_ref_lane_id <<
  // std::endl;

  std::vector<int> potential_lane_ids;
  // 候选包括目标相邻 Lane 及其 child Lane。
  GetPotentialLaneIds(action_ref_lane_id, lat_behavior, &potential_lane_ids);

  // 只按 Lane ID 判定，不额外检查横向偏差、航向或稳定保持时间。
  auto it = std::find(potential_lane_ids.begin(), potential_lane_ids.end(),
                      *current_lane_id);
  if (it == potential_lane_ids.end()) {
    return false;
  } else {
    return true;
  }
}

// 执行 EUDM 单周期：读取输入、建立 RSS 参考 Lane、预删非法脚本、仿真并记录胜者。
ErrorType EudmPlanner::RunOnce() {
  TicToc timer_runonce;
  // 地图接口和自车是本周期的首要前置条件。
  if (!map_itf_) {
    LOG(ERROR) << "[Eudm]map interface not initialized. Exit";
    return kWrongStatus;
  }

  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Eudm]no ego vehicle found.";
    return kWrongStatus;
  }
  ego_id_ = ego_vehicle_.id();
  time_stamp_ = ego_vehicle_.state().time_stamp;

  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]------ Eudm Cycle Begins (stamp): " << time_stamp_
               << " ------- ";

  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(std::vector<int>(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    LOG(ERROR) << "[Eudm]Fatal (Exit) ego not on lane.";
    return kWrongStatus;
  }
  LOG(WARNING) << std::fixed << std::setprecision(3)
               << "[Eudm][Input]Ego plan state (x,y,theta,v,a,k):("
               << ego_vehicle_.state().vec_position[0] << ","
               << ego_vehicle_.state().vec_position[1] << ","
               << ego_vehicle_.state().angle << ","
               << ego_vehicle_.state().velocity << ","
               << ego_vehicle_.state().acceleration << ","
               << ego_vehicle_.state().curvature << ")"
               << " lane id:" << ego_lane_id_by_pos;
  LOG(WARNING) << "[Eudm][Setup]Desired vel:" << desired_velocity_
               << " sim_time total:" << sim_time_total_
               << " lc info[f_l,f_r,us_ol,us_or,solid_l,solid_r]:"
               << lc_info_.forbid_lane_change_left << ","
               << lc_info_.forbid_lane_change_right << ","
               << lc_info_.lane_change_left_unsafe_by_occu << ","
               << lc_info_.lane_change_right_unsafe_by_occu << ","
               << lc_info_.left_solid_lane << "," << lc_info_.right_solid_lane;
  ego_lane_id_ = ego_lane_id_by_pos;

  // 构造固定前后 130 m 的 LK 参考 Lane，供本周期 RSS 评价使用。
  const decimal_t forward_rss_check_range = 130.0;
  const decimal_t backward_rss_check_range = 130.0;
  const decimal_t forward_lane_len = forward_rss_check_range;
  const decimal_t backward_lane_len = backward_rss_check_range;
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_.state(), std::vector<int>(),
          LateralBehavior::kLaneKeeping, forward_lane_len, backward_lane_len,
          false, &rss_lane_) != kSuccess) {
    LOG(ERROR) << "[Eudm]No Rss lane available. Rss disabled";
  }

  // 查询成功或历史 rss_lane_ 仍有效时刷新 Frenet 变换。
  if (rss_lane_.IsValid()) {
    rss_stf_ = common::StateTransformer(rss_lane_);
  }

  // 预删相邻层直接从 LCL 切到 LCR 或反向切换的脚本。
  pre_deleted_seq_ids_.clear();
  int n_sequence = dcp_tree_ptr_->action_script().size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    int num_actions = action_seq.size();
    for (int j = 1; j < num_actions; j++) {
      if ((action_seq[j - 1].lat == DcpLatAction::kLaneChangeLeft &&
           action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)) {
        pre_deleted_seq_ids_.insert(i);
      }
    }
  }

  // 执行并行候选仿真与最小代价选择。
  TicToc timer;
  if (RunEudm() != kSuccess) {
    LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]****** Eudm Cycle FAILED (stamp): " << time_stamp_
               << " time cost " << timer.toc() << " ms.";
    return kWrongStatus;
  }
  // 记录胜出脚本的人类可读动作缩写和本周期耗时。
  auto action_script = dcp_tree_ptr_->action_script();
  std::ostringstream line_info;
  line_info << "[Eudm]SUCCESS id:" << winner_id_ << " [";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLonActionName(a.lon);
  }
  line_info << "|";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3) << winner_score_
            << " time cost: " << timer.toc() << " ms.";
  LOG(WARNING) << line_info.str();

  time_cost_ = timer_runonce.toc();
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSinglePolicyTrajs(
    const std::vector<CostStructure>& progress_cost,
    const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
    decimal_t* score) {
  decimal_t score_tmp = 0.0;
  for (const auto& c : progress_cost) {
    score_tmp += c.ave();
  }
  *score = score_tmp + tail_cost.ave();
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateMultiThreadSimResults(int* winner_id,
                                                     decimal_t* winner_cost) {
  decimal_t min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();
  for (int i = 0; i < num_sequences; ++i) {
    if (sim_res_[i] == 0) {
      continue;
    }
    decimal_t cost = 0.0;
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    EvaluateSinglePolicyTrajs(progress_cost_[i], tail_cost_[i], action_seq,
                              &cost);
    final_cost_[i] = cost;
    if (cost < min_cost) {
      min_cost = cost;
      best_id = i;
    }
  }
  *winner_cost = min_cost;
  *winner_id = best_id;
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSafetyStatus(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost, bool* is_rss_safe, int* risky_id) {
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }
  if (!cfg_.safety().rss_check_enable() || !rss_lane_.IsValid()) {
    return kSuccess;
  }
  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;
  bool ret_is_rss_safe = true;
  const int check_per_state = 1;
  for (int i = 0; i < num_states; i += check_per_state) {
    bool is_rss_safe = true;
    common::RssChecker::LongitudinalViolateType type;
    decimal_t rss_vel_low, rss_vel_up;
    common::RssChecker::RssCheck(traj_a[i], traj_b[i], rss_stf_, rss_config_,
                                 &is_rss_safe, &type, &rss_vel_low,
                                 &rss_vel_up);
    if (!is_rss_safe) {
      ret_is_rss_safe = false;
      *risky_id = traj_b.size() ? traj_b[0].id() : 0;
      if (cfg_.cost().safety().rss_cost_enable()) {
        if (type == common::RssChecker::LongitudinalViolateType::TooFast) {
          cost_tmp +=
              cfg_.cost().safety().rss_over_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_over_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_up));
        } else if (type ==
                   common::RssChecker::LongitudinalViolateType::TooSlow) {
          cost_tmp +=
              cfg_.cost().safety().rss_lack_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_lack_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_low));
        }
      }
    }
  }
  *is_rss_safe = ret_is_rss_safe;
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType EudmPlanner::StrictSafetyCheck(
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool* is_safe, int* collided_id) {
  if (!cfg_.safety().strict_check_enable()) {
    *is_safe = true;
    return kSuccess;
  }

  int num_points_ego = ego_traj.size();
  if (num_points_ego == 0) {
    *is_safe = true;
    return kSuccess;
  }
  // strict collision check
  for (auto it = surround_trajs.begin(); it != surround_trajs.end(); it++) {
    int num_points_other = it->second.size();
    if (num_points_other != num_points_ego) {
      *is_safe = false;
      LOG(ERROR) << "[Eudm]unsafe due to incomplete sim record for vehicle";
      return kSuccess;
    }
    for (int i = 0; i < num_points_ego; i++) {
      common::Vehicle inflated_a, inflated_b;
      common::SemanticsUtils::InflateVehicleBySize(
          ego_traj[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_a);
      common::SemanticsUtils::InflateVehicleBySize(
          it->second[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_b);
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                         inflated_b.param(), inflated_b.state(),
                                         &is_collision);
      if (is_collision) {
        *is_safe = false;
        *collided_id = it->second[i].id();
        return kSuccess;
      }
    }
  }
  *is_safe = true;
  return kSuccess;
}

ErrorType EudmPlanner::CostFunction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
    const ForwardSimAgentSet& other_fsagent,
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool verbose, CostStructure* cost, bool* is_risky,
    std::set<int>* risky_ids) {
  decimal_t duration = action.t;

  auto ego_lon_behavior_this_layer = ego_fsagent.lon_behavior;
  auto ego_lat_behavior_this_layer = ego_fsagent.lat_behavior;

  auto seq_lat_behavior = ego_fsagent.seq_lat_behavior;
  auto is_cancel_behavior = ego_fsagent.is_cancel_behavior;

  common::VehicleSet vehicle_set;
  for (const auto& v : other_fsagent.forward_sim_agents) {
    vehicle_set.vehicles.insert(std::make_pair(v.first, v.second.vehicle));
  }

  decimal_t ego_velocity = ego_fsagent.vehicle.state().velocity;
  // f = c1 * fabs(v_ego - v_user), if v_ego < v_user
  // f = c2 * fabs(v_ego - v_user - vth), if v_ego > v_user + vth
  // unit of this cost is velocity (finally multiplied by duration)
  CostStructure cost_tmp;
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_) {
    cost_tmp.efficiency.ego_to_desired_vel =
        cfg_.cost().effciency().ego_lack_speed_to_desired_unit_cost() *
        fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_);
  } else {
    if (ego_fsagent.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost().effciency().ego_desired_speed_tolerate_gap()) {
      cost_tmp.efficiency.ego_to_desired_vel =
          cfg_.cost().effciency().ego_over_speed_to_desired_unit_cost() *
          fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost().effciency().ego_desired_speed_tolerate_gap());
    }
  }

  // f = ratio * c1 * fabs(v_ego - v_user) , if v_ego < v_user && v_ego
  // > v_leading
  // unit of this cost is velocity (finally multiplied by duration)
  common::Vehicle leading_vehicle;
  decimal_t distance_residual_ratio = 0.0;
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_fsagent.target_lane, ego_fsagent.vehicle.state(), vehicle_set,
          ego_fsagent.lat_range, &leading_vehicle,
          &distance_residual_ratio) == kSuccess) {
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_fsagent.vehicle.state().vec_position)
            .norm();
    if (ego_fsagent.vehicle.state().velocity < desired_velocity_ &&
        leading_vehicle.state().velocity < desired_velocity_ &&
        distance_to_leading_vehicle <
            cfg_.cost().effciency().leading_distance_th()) {
      decimal_t ego_blocked_by_leading_velocity =
          ego_fsagent.vehicle.state().velocity >
                  leading_vehicle.state().velocity
              ? ego_fsagent.vehicle.state().velocity -
                    leading_vehicle.state().velocity
              : 0.0;
      decimal_t leading_to_desired_velocity =
          leading_vehicle.state().velocity < desired_velocity_
              ? desired_velocity_ - leading_vehicle.state().velocity
              : 0.0;
      cost_tmp.efficiency.leading_to_desired_vel =
          std::max(cfg_.cost().effciency().min_distance_ratio(),
                   distance_residual_ratio) *
          (cfg_.cost().effciency().ego_speed_blocked_by_leading_unit_cost() *
               ego_blocked_by_leading_velocity +
           cfg_.cost()
                   .effciency()
                   .leading_speed_blocked_desired_vel_unit_cost() *
               leading_to_desired_velocity);
    }
  }

  // * safety
  for (const auto& surround_traj : surround_trajs) {
    decimal_t safety_cost = 0.0;
    bool is_safe = true;
    int risky_id = 0;
    EvaluateSafetyStatus(ego_traj, surround_traj.second, &safety_cost, &is_safe,
                         &risky_id);
    if (!is_safe) {
      risky_ids->insert(risky_id);
      *is_risky = true;
    }
    cost_tmp.safety.rss += safety_cost;
  }

  if (cfg_.cost().safety().occu_lane_enable()) {
    if (lc_info_.forbid_lane_change_left &&
        seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    } else if (lc_info_.forbid_lane_change_right &&
               seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    }
  }

  // * navigation
  if (seq_lat_behavior == LateralBehavior::kLaneChangeLeft ||
      seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
    if (is_cancel_behavior) {
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          cfg_.cost().user().cancel_operation_unit_cost();
    } else {
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          (seq_lat_behavior == LateralBehavior::kLaneChangeLeft
               ? cfg_.cost().navigation().lane_change_left_unit_cost()
               : cfg_.cost().navigation().lane_change_right_unit_cost());
      if (lc_info_.recommend_lc_left &&
          seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_left_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeLeft) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      } else if (lc_info_.recommend_lc_right &&
                 seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_right_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeRight) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      }
    }
  }
  cost_tmp.weight = duration;
  *cost = cost_tmp;
  return kSuccess;
}

// 把动作时长拆成若干固定 step，并把不足一个 step 的余数放在首步。
ErrorType EudmPlanner::GetSimTimeSteps(const DcpAction& action,
                                       std::vector<decimal_t>* dt_steps) const {
  decimal_t sim_time_resolution = cfg_.sim().duration().step();
  decimal_t sim_time_total = action.t;
  int n_1 = std::floor(sim_time_total / sim_time_resolution);
  decimal_t dt_remain = sim_time_total - n_1 * sim_time_resolution;
  std::vector<decimal_t> steps(n_1, sim_time_resolution);
  if (fabs(dt_remain) > kEPS) {
    steps.insert(steps.begin(), dt_remain);
  }
  *dt_steps = steps;

  return kSuccess;
}

// 从同一旧交通快照同步计算自车和全部周车的下一步，再统一提交状态。
ErrorType EudmPlanner::SimulateSingleAction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
    const ForwardSimAgentSet& surrounding_fsagents_this_layer,
    vec_E<common::Vehicle>* ego_traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // 输出容器每层重新清空，并为每个周车建立轨迹槽位。
  ego_traj->clear();
  surround_trajs->clear();
  for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
  }

  // 动作时长按配置分辨率离散；当前不校验零/负 step 或动作时长。
  std::vector<decimal_t> dt_steps;
  GetSimTimeSteps(action, &dt_steps);

  ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;
  ForwardSimAgentSet surrounding_fsagents_this_step =
      surrounding_fsagents_this_layer;

  // 每个积分步先构造旧状态交通快照，再分别计算所有 agent 的新状态。
  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    decimal_t sim_time_step = dt_steps[i];

    common::State ego_state_cache_this_step;
    std::unordered_map<int, State>
        others_state_cache_this_step;  // 周车 ID -> 本步输出状态。

    common::VehicleSet all_sim_vehicles;  // 同时包含自车和全部周车。
    all_sim_vehicles.vehicles.insert(std::make_pair(
        ego_fsagent_this_step.vehicle.id(), ego_fsagent_this_step.vehicle));
    for (const auto& v : surrounding_fsagents_this_step.forward_sim_agents) {
      all_sim_vehicles.vehicles.insert(
          std::make_pair(v.first, v.second.vehicle));
    }

    // 传播自车时把自身 ID 临时改为 invalid，避免前车搜索命中自身。
    {
      all_sim_vehicles.vehicles.at(ego_id_).set_id(kInvalidAgentId);

      common::State state_output;
      if (kSuccess != EgoAgentForwardSim(ego_fsagent_this_step,
                                         all_sim_vehicles, sim_time_step,
                                         &state_output)) {
        return kWrongStatus;
      }

      common::Vehicle v_tmp = ego_fsagent_this_step.vehicle;
      v_tmp.set_state(state_output);
      ego_traj->push_back(v_tmp);
      ego_state_cache_this_step = state_output;

      all_sim_vehicles.vehicles.at(ego_id_).set_id(ego_id_);
    }

    // 逐个传播周车；每辆车计算时只隐藏自己，其他 agent 均保持旧状态。
    {
      for (const auto& p_fsa :
           surrounding_fsagents_this_step.forward_sim_agents) {
        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(kInvalidAgentId);

        common::State state_output;
        if (kSuccess !=
            SurroundingAgentForwardSim(p_fsa.second, all_sim_vehicles,
                                       sim_time_step, &state_output)) {
          return kWrongStatus;
        }

        common::Vehicle v_tmp = p_fsa.second.vehicle;
        v_tmp.set_state(state_output);
        surround_trajs->at(p_fsa.first).push_back(v_tmp);
        others_state_cache_this_step.insert(
            std::make_pair(p_fsa.first, state_output));

        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(p_fsa.first);
      }
    }

    // 所有下一状态计算完成后同步提交，作为下一积分步的输入。
    ego_fsagent_this_step.vehicle.set_state(ego_state_cache_this_step);
    for (const auto& ps : others_state_cache_this_step) {
      surrounding_fsagents_this_step.forward_sim_agents.at(ps.first)
          .vehicle.set_state(ps.second);
    }
  }

  return kSuccess;
}

// 在固定参考 Lane 上按 IDM/运动学模型传播单个周车。
ErrorType EudmPlanner::SurroundingAgentForwardSim(
    const ForwardSimAgent& fsagent, const common::VehicleSet& all_sim_vehicles,
    const decimal_t& sim_time_step, common::State* state_out) const {
  common::Vehicle leading_vehicle;
  common::State state_output;
  decimal_t distance_residual_ratio = 0.0;
  // 未找到前车时保留默认 invalid Vehicle，交由传播器解释自由行驶。
  map_itf_->GetLeadingVehicleOnLane(fsagent.lane, fsagent.vehicle.state(),
                                    all_sim_vehicles, fsagent.lat_range,
                                    &leading_vehicle, &distance_residual_ratio);
  if (planning::OnLaneForwardSimulation::PropagateOnce(
          fsagent.stf, fsagent.vehicle, leading_vehicle, sim_time_step,
          fsagent.sim_param, &state_output) != kSuccess) {
    return kWrongStatus;
  }
  *state_out = state_output;
  return kSuccess;
}

// 根据 LK/LC 模式选择自车传播器，并在换道时加入目标 gap 与避让参数。
ErrorType EudmPlanner::EgoAgentForwardSim(
    const ForwardSimEgoAgent& ego_fsagent,
    const common::VehicleSet& all_sim_vehicles, const decimal_t& sim_time_step,
    common::State* state_out) const {
  common::State state_output;

  if (ego_fsagent.lat_behavior == LateralBehavior::kLaneKeeping) {
    // LK 只搜索目标 Lane 前车，并在传播前拒绝已经发生的车身碰撞。
    common::Vehicle leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.target_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // 找到前车时执行即时碰撞检查。
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          leading_vehicle.param(), leading_vehicle.state(), &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // TODO(lu.zhang): 使用横向社会力生成动态跟踪偏移。
    decimal_t lat_track_offset = 0.0;
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLK(
            ego_fsagent.target_stf, ego_fsagent.vehicle, leading_vehicle,
            lat_track_offset, sim_time_step, ego_fsagent.sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    // LC 同时考虑当前 Lane 前车以及目标 gap 的前/后车。
    common::Vehicle current_leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.current_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &current_leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // 当前 Lane 前车存在时先做即时碰撞检查。
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          current_leading_vehicle.param(), current_leading_vehicle.state(),
          &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // gap ID 在动作层开始时选定；-1 表示该方向没有边界车辆。
    common::Vehicle gap_front_vehicle;
    if (ego_fsagent.target_gap_ids(0) != -1) {
      gap_front_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(0));
    }
    common::Vehicle gap_rear_vehicle;
    if (ego_fsagent.target_gap_ids(1) != -1) {
      gap_rear_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(1));
    }

    // TODO(lu.zhang): 使用横向社会力生成动态跟踪偏移。
    decimal_t lat_track_offset = 0.0;
    auto sim_param = ego_fsagent.sim_param;

    if (gap_rear_vehicle.id() != kInvalidAgentId &&
        cfg_.sim().ego().evasive().evasive_enable()) {
      common::FrenetState ego_on_tarlane_fs;
      common::FrenetState rear_on_tarlane_fs;
      if (ego_fsagent.target_stf.GetFrenetStateFromState(
              ego_fsagent.vehicle.state(), &ego_on_tarlane_fs) == kSuccess &&
          ego_fsagent.target_stf.GetFrenetStateFromState(
              gap_rear_vehicle.state(), &rear_on_tarlane_fs) == kSuccess) {
        // 后车 RSS 不安全且自车过慢时，提高 IDM 期望速度和加速能力。
        bool is_rss_safe = true;
        common::RssChecker::LongitudinalViolateType type;
        decimal_t rss_vel_low, rss_vel_up;
        common::RssChecker::RssCheck(ego_fsagent.vehicle, gap_rear_vehicle,
                                     ego_fsagent.target_stf,
                                     rss_config_strict_as_rear_, &is_rss_safe,
                                     &type, &rss_vel_low, &rss_vel_up);
        if (!is_rss_safe) {
          if (type == common::RssChecker::LongitudinalViolateType::TooSlow) {
            sim_param.idm_param.kDesiredVelocity = std::max(
                sim_param.idm_param.kDesiredVelocity,
                rss_vel_low + cfg_.sim().ego().evasive().lon_extraspeed());
            sim_param.idm_param.kDesiredHeadwayTime =
                cfg_.sim().ego().evasive().head_time();
            sim_param.idm_param.kAcceleration =
                cfg_.sim().ego().evasive().lon_acc();
            sim_param.max_lon_acc_jerk = cfg_.sim().ego().evasive().lon_jerk();
          }
        }
        // virtual barrier 在 gap 过窄时保持当前横向偏移，暂缓继续横移。
        if (cfg_.sim().ego().evasive().virtual_barrier_enable()) {
          if (ego_on_tarlane_fs.vec_s[0] - rear_on_tarlane_fs.vec_s[0] <
              gap_rear_vehicle.param().length() +
                  cfg_.sim().ego().evasive().virtual_barrier_tic() *
                      gap_rear_vehicle.state().velocity) {
            lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
          }
          common::FrenetState front_on_tarlane_fs;
          if (gap_front_vehicle.id() != kInvalidAgentId &&
              ego_fsagent.target_stf.GetFrenetStateFromState(
                  gap_front_vehicle.state(), &front_on_tarlane_fs) ==
                  kSuccess) {
            if (front_on_tarlane_fs.vec_s[0] - ego_on_tarlane_fs.vec_s[0] <
                ego_fsagent.vehicle.param().length() +
                    cfg_.sim().ego().evasive().virtual_barrier_tic() *
                        ego_fsagent.vehicle.state().velocity) {
              lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
            }
          }
        }
      }
    }

    // LC 传播器联合当前 Lane 前车、目标 gap 前后车和避让后的参数。
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLC(
            ego_fsagent.current_stf, ego_fsagent.target_stf,
            ego_fsagent.vehicle, current_leading_vehicle, gap_front_vehicle,
            gap_rear_vehicle, lat_track_offset, sim_time_step, sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  }

  *state_out = state_output;

  return kSuccess;
}

// 根据当前 Lane 是否落在保持/左换/右换候选集中推断横向行为。
ErrorType EudmPlanner::JudgeBehaviorByLaneId(
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
    // 若未来接入 routing，还需验证该 Lane 延续是否与导航路径一致。
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

// 更新自车 Lane ID；潜在 Lane 集合的同步刷新当前已被注释。
ErrorType EudmPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
  //                     &potential_lk_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeLeft,
  //                     &potential_lcl_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_,
  // common::LateralBehavior::kLaneChangeRight,
  //                     &potential_lcr_lane_ids_);
  return kSuccess;
}

// 由源 Lane 和横向行为构造可能到达的相邻 Lane/child Lane 集合。
ErrorType EudmPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) const {
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

// 保存由 EudmManager 持有的非拥有地图接口指针。
void EudmPlanner::set_map_interface(EudmPlannerMapItf* itf) { map_itf_ = itf; }

// 把用户期望速度下限截断为零。
void EudmPlanner::set_desired_velocity(const decimal_t desired_vel) {
  desired_velocity_ = std::max(0.0, desired_vel);
}

// 按值缓存本周期换道约束和推荐信息。
void EudmPlanner::set_lane_change_info(const LaneChangeInfo& lc_info) {
  lc_info_ = lc_info;
}

// 返回当前用户期望速度。
decimal_t EudmPlanner::desired_velocity() const { return desired_velocity_; }

// 返回本周期胜出候选索引。
int EudmPlanner::winner_id() const { return winner_id_; }

// 返回最近一次成功 RunOnce 的总耗时。
decimal_t EudmPlanner::time_cost() const { return time_cost_; }

// 用 manager 给出的 ongoing action 重建 DCP 候选并刷新规划时域。
void EudmPlanner::UpdateDcpTree(const DcpAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->UpdateScript();
  sim_time_total_ = dcp_tree_ptr_->planning_horizon();
}

// 返回非拥有地图接口指针。
EudmPlannerMapItf* EudmPlanner::map_itf() const { return map_itf_; }

}  // namespace planning
