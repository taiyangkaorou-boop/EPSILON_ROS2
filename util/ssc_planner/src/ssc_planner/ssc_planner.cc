/**
 * @file ssc_planner.cc
 * @author HKUST Aerial Robotics Group
 * @brief SSC 规划器的数据准备、走廊构建和轨迹优化实现。
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include "ssc_planner/ssc_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <omp.h>

// 多核任务调度会显著影响延迟；当前默认关闭 OpenMP 路径。
#define USE_OPENMP 0

namespace planning {

std::string SscPlanner::Name() { return std::string("ssc_planner"); }

ErrorType SscPlanner::Init(const std::string config_path) {
  // 先解析 protobuf 文本；原始实现不检查 ReadConfig 返回值。
  ReadConfig(config_path);

  // 打印规划器层的低速切换阈值和参考轨迹贴近权重。
  printf("\nSscPlanner Config:\n");
  printf(" -- weight_proximity: %lf\n", cfg_.planner_cfg().weight_proximity());

  LOG(INFO) << "[Ssc]SscPlanner Config:";
  LOG(INFO) << "[Ssc] -- low spd threshold: "
            << cfg_.planner_cfg().low_speed_threshold();
  LOG(INFO) << "[Ssc] -- weight_proximity: "
            << cfg_.planner_cfg().weight_proximity();

  // 将 protobuf MapConfig 逐字段映射到运行时 SscMap::Config。
  SscMap::Config map_cfg;
  map_cfg.map_size[0] = cfg_.map_cfg().map_size_x();
  map_cfg.map_size[1] = cfg_.map_cfg().map_size_y();
  map_cfg.map_size[2] = cfg_.map_cfg().map_size_z();
  map_cfg.map_resolution[0] = cfg_.map_cfg().map_resl_x();
  map_cfg.map_resolution[1] = cfg_.map_cfg().map_resl_y();
  map_cfg.map_resolution[2] = cfg_.map_cfg().map_resl_z();
  map_cfg.s_back_len = cfg_.map_cfg().s_back_len();
  // 最小纵向速度不低于求解器的速度奇异点保护阈值。
  map_cfg.kMaxLongitudinalVel = cfg_.map_cfg().dyn_bounds().max_lon_vel();
  map_cfg.kMinLongitudinalVel =
      std::max(cfg_.map_cfg().dyn_bounds().min_lon_vel(),
               cfg_.planner_cfg().velocity_singularity_eps());
  map_cfg.kMaxLongitudinalAcc = cfg_.map_cfg().dyn_bounds().max_lon_acc();
  map_cfg.kMaxLongitudinalDecel = cfg_.map_cfg().dyn_bounds().max_lon_dec();
  map_cfg.kMaxLateralVel = cfg_.map_cfg().dyn_bounds().max_lat_vel();
  map_cfg.kMaxLateralAcc = cfg_.map_cfg().dyn_bounds().max_lat_acc();
  map_cfg.kMaxNumOfGridAlongTime = cfg_.map_cfg().max_grids_along_time();
  map_cfg.inflate_steps[0] = cfg_.map_cfg().infl_steps().x_p();
  map_cfg.inflate_steps[1] = cfg_.map_cfg().infl_steps().x_n();
  map_cfg.inflate_steps[2] = cfg_.map_cfg().infl_steps().y_p();
  map_cfg.inflate_steps[3] = cfg_.map_cfg().infl_steps().y_n();
  map_cfg.inflate_steps[4] = cfg_.map_cfg().infl_steps().z_p();
  map_cfg.inflate_steps[5] = cfg_.map_cfg().infl_steps().z_n();
  // SscPlanner 持有该裸指针，但类中没有对应释放逻辑。
  p_ssc_map_ = new SscMap(map_cfg);

  return kSuccess;
}

ErrorType SscPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading ssc planner config\n");
  using namespace google::protobuf;
  // 以 POSIX fd 构造 protobuf 零拷贝输入流并解析文本格式配置。
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  TextFormat::Parse(&fstream, &cfg_);
  // 缺少 required 字段时记录错误并触发断言；函数本身仍声明固定成功返回。
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

ErrorType SscPlanner::set_initial_state(const State& state) {
  // 该标志只影响下一次 RunOnce，使用后会自动复位。
  initial_state_ = state;
  has_initial_state_ = true;
  return kSuccess;
}

ErrorType SscPlanner::RunOnce() {
  // 本轮时间戳直接来自地图快照；调用前假定 map_itf_ 已正确绑定。
  stamp_ = map_itf_->GetTimeStamp();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE START: " << stamp_
               << " ********************\n";
  static TicToc ssc_timer;
  ssc_timer.tic();

  static TicToc timer_prepare;
  timer_prepare.tic();
  // 第一阶段：从同一地图快照取得自车和行为/环境输入。
  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego vehicle info.";
    return kWrongStatus;
  }

  // 未显式设置规划起点时使用当前自车；显式状态仅消费一次。
  if (!has_initial_state_) {
    initial_state_ = ego_vehicle_.state();
  }
  has_initial_state_ = false;

  // 高于阈值使用 s/d 独立 Bezier 优化，低速使用 Frenet primitive 连接。
  is_lateral_independent_ =
      initial_state_.velocity > cfg_.planner_cfg().low_speed_threshold()
          ? true
          : false;
  if (map_itf_->GetLocalReferenceLane(&nav_lane_local_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to find ego lane.";
    return kWrongStatus;
  }
  // 所有后续状态和障碍点都投影到该局部参考 Lane 的 Frenet 坐标系。
  stf_ = common::StateTransformer(nav_lane_local_);

  if (stf_.GetFrenetStateFromState(initial_state_, &initial_frenet_state_) !=
      kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get init state frenet state.";
    return kWrongStatus;
  }

  if (map_itf_->GetEgoDiscretBehavior(&ego_behavior_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego behavior.";
    return kWrongStatus;
  }

  if (map_itf_->GetObstacleMap(&grid_map_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle map.";
    return kWrongStatus;
  }

  if (map_itf_->GetObstacleGrids(&obstacle_grids_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle grids.";
    return kWrongStatus;
  }

  if (map_itf_->GetForwardTrajectories(&forward_behaviors_, &forward_trajs_,
                                       &surround_forward_trajs_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get forward trajectories.";
    return kWrongStatus;
  }

  auto t_prepare = timer_prepare.toc();
  LOG(WARNING) << "[Ssc]prepare time cost: " << t_prepare << " ms";

  static TicToc timer_stf;
  timer_stf.tic();
  // 第二阶段：批量转换起点、全部 rollout、车身顶点和静态障碍点。
  if (StateTransformForInputData() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to transform state into ff.";
    return kWrongStatus;
  }
  auto t_stf = timer_stf.toc();
  LOG(WARNING) << "[Ssc]state transform time cost: " << t_stf << " ms";

  // 第三阶段耗时可能受 CPU 调度影响，单独统计 SscMap 构建时间。
  static TicToc timer_sscmap;
  timer_sscmap.tic();
  time_origin_ = initial_state_.time_stamp;
  // 新规划周期共享同一初始 Frenet 原点，并清空旧走廊/栅格。
  p_ssc_map_->ResetSscMap(initial_frenet_state_);
  // 每个行为使用其专属周车闭环 rollout 构建占用，再沿对应自车 rollout 提取走廊。
  int num_behaviors = forward_behaviors_.size();
  for (int i = 0; i < num_behaviors; ++i) {
    if (!cfg_.planner_cfg().is_fitting_only()) {
      if (p_ssc_map_->ConstructSscMap(surround_forward_trajs_fs_[i],
                                      obstacle_grids_fs_)) {
        LOG(ERROR) << "[Ssc]fail to construct ssc map.";
        return kWrongStatus;
      }
    }
    // EUDM 集成当前为节省时间关闭自车足迹障碍膨胀，走廊直接使用原始占用图。
    // TicToc timer_infl;
    // p_ssc_map_->InflateObstacleGrid(ego_vehicle_.param());
    // printf("[SscPlanner] InflateObstacleGrid time cost: %lf ms\n",
    //        timer_infl.toc());
    if (p_ssc_map_->ConstructCorridorUsingInitialTrajectory(
            p_ssc_map_->p_3d_grid(), forward_trajs_fs_[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail to construct corridor for behavior " << i;
      return kWrongStatus;
    }
  }
  // 将所有候选离散 DrivingCorridor 转换为优化器读取的连续时空 cube。
  if (kSuccess != p_ssc_map_->GetFinalGlobalMetricCubesList()) {
    LOG(ERROR) << "[Ssc]fail to get final corridor";
    return kWrongStatus;
  }
  auto t_sscmap = timer_sscmap.toc();
  LOG(WARNING) << "[Ssc]construct ssc map and corridor time cost: " << t_sscmap
               << " ms";

  static TicToc timer_opt;
  timer_opt.tic();
  // 第四阶段：逐候选求解轨迹，并选择与当前行为匹配的最终输出。
  if (RunQpOptimization() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to optimize qp trajectories.\n";
    return kWrongStatus;
  }

  if (UpdateTrajectoryWithCurrentBehavior() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail: current behavior "
               << static_cast<int>(ego_behavior_) << " not valid.";
    LOG(ERROR) << "[Ssc]fail: has " << qp_trajs_.size() << " traj, "
               << valid_behaviors_.size() << " behaviors.";
    return kWrongStatus;
  }

#if 0
  // 原始轨迹可行性验证被编译期开关禁用。
  auto traj = trajectory();
  if (ValidateTrajectory(*traj) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail: infeasible traj.";
    return kWrongStatus;
  }
#endif

  auto t_opt = timer_opt.toc();
  LOG(WARNING) << "[Ssc]optimization time cost: " << t_opt << " ms";

  auto t_sum = t_prepare + t_stf + t_sscmap + t_opt;
  // 同时记录各阶段和整轮墙钟耗时，diff 表示未单独归类的开销。
  time_cost_ = ssc_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]Sum of time: " << t_sum
               << " ms, diff: " << time_cost_ - t_sum << " ms";
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE FINISH: " << stamp_ << " +"
               << time_cost_ << " ms ********************\n";

  return kSuccess;
}  // SscPlanner::RunOnce

ErrorType SscPlanner::RunQpOptimization() {
  // 按值取得候选连续走廊和有效标志，预期与 forward_behaviors_ 同下标对齐。
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> cube_list =
      p_ssc_map_->final_corridor_vec();
  std::vector<int> if_corridor_valid = p_ssc_map_->if_corridor_valid();
  if (cube_list.empty()) return kWrongStatus;
  // 当前只显式校验 corridor 数量，不校验有效标志或 Frenet rollout 数量。
  if (cube_list.size() != forward_behaviors_.size()) {
    LOG(ERROR) << "[Ssc]cube list " << static_cast<int>(cube_list.size())
               << " not consist with behavior size: "
               << static_cast<int>(forward_behaviors_.size())
               << ", forward traj " << static_cast<int>(forward_trajs_.size())
               << ", flag size " << static_cast<int>(if_corridor_valid.size());
    return kWrongStatus;
  }

  // 每轮完全重建通过筛选的轨迹、走廊、参考状态和行为容器。
  qp_trajs_.clear();
  primitive_trajs_.clear();
  valid_behaviors_.clear();
  corridors_.clear();
  ref_states_list_.clear();
  for (int i = 0; i < static_cast<int>(cube_list.size()); i++) {
    // beh 仅保留原始离散行为数值，当前后续没有使用。
    int beh = static_cast<int>(forward_behaviors_[i]);
    // 无有效 SSC 的候选直接跳过，不进入优化。
    if (if_corridor_valid[i] == 0) {
      LOG(ERROR) << "[Ssc]fail: for behavior "
                 << static_cast<int>(forward_behaviors_[i])
                 << " has no valid corridor.";
      continue;
    }

    // 取该行为的自车 Frenet rollout 作为终端约束和 proximity 参考。
    auto fs_vehicle_traj = forward_trajs_fs_[i];
    int num_states = static_cast<int>(fs_vehicle_traj.size());

    vec_E<Vecf<2>> start_constraints;
    // 起点依次约束 s/d 位置、速度和加速度；纵向速度夹到奇异保护阈值以上。
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[0], ego_frenet_state_.vec_dt[0]));
    start_constraints.push_back(
        Vecf<2>(std::max(ego_frenet_state_.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                ego_frenet_state_.vec_dt[1]));
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[2], ego_frenet_state_.vec_dt[2]));

    // 保留的起点一致性调试输出默认关闭。
    // printf("[Inconsist]Start sd position (%lf, %lf).\n",
    // start_constraints[0](0),
    //        start_constraints[0](1));
    // 终点只约束最后 rollout 状态的 s/d 位置和速度，不约束加速度。
    vec_E<Vecf<2>> end_constraints;
    end_constraints.push_back(
        Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[0],
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[0]));
    end_constraints.push_back(
        Vecf<2>(std::max(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[1]));
    // end_constraints.push_back(
    //     Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[2],
    //             fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[2]));
    common::SplineGenerator<5, 2> spline_generator;
    BezierSpline bezier_spline;

    // 把最后一个 corridor 的终止时间强制对齐参考 rollout 终点。
    cube_list[i].back().t_ub = fs_vehicle_traj.back().frenet_state.time_stamp;

    // 当前可行性检查只验证非空和相邻 cube 时间边界连续。
    if (CorridorFeasibilityCheck(cube_list[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail: corridor not valid for optimization.";
      continue;
    }

    std::vector<decimal_t> ref_stamps;
    vec_E<Vecf<2>> ref_points;
    vec_E<common::FrenetState> ref_states;
    // 完整 rollout 的时间、s/d 位置和 Frenet 状态作为 QP proximity 参考及可视化依据。
    for (int n = 0; n < num_states; n++) {
      ref_stamps.push_back(fs_vehicle_traj[n].frenet_state.time_stamp);
      ref_points.push_back(Vecf<2>(fs_vehicle_traj[n].frenet_state.vec_s[0],
                                   fs_vehicle_traj[n].frenet_state.vec_dt[0]));
      ref_states.push_back(fs_vehicle_traj[n].frenet_state);
    }

    bool bezier_spline_gen_success = true;
    // 五阶二维 spline 同时满足 corridor、起终约束，并按权重贴近参考点。
    if (spline_generator.GetBezierSplineUsingCorridor(
            cube_list[i], start_constraints, end_constraints, ref_stamps,
            ref_points, cfg_.planner_cfg().weight_proximity(),
            &bezier_spline) != kSuccess) {
      if (is_lateral_independent_) {
        // 正常速度求解失败时输出走廊、参考点、原轨迹和边界约束诊断信息。
        LOG(ERROR) << "[Ssc]fail: solver error for behavior "
                   << static_cast<int>(forward_behaviors_[i]);
        decimal_t t0 = cube_list[i].front().t_lb;
        for (auto& cube : cube_list[i]) {
          LOG(ERROR) << std::fixed << std::setprecision(3) << "[Ssc] t: ["
                     << cube.t_lb - t0 << ", " << cube.t_ub - t0 << "], x: ["
                     << cube.p_lb[0] << ", " << cube.p_ub[0] << "], y: ["
                     << cube.p_lb[1] << ", " << cube.p_ub[1] << "]";
        }
        LOG(ERROR) << "[Ssc]ref points: ";
        for (int k = 0; k < ref_stamps.size(); ++k) {
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << ref_stamps[k] << ", x: " << ref_points[k].x()
                     << ", y: " << ref_points[k].y();
        }
        LOG(ERROR) << "[Ssc]forward traj: ";
        for (int k = 0; k < forward_trajs_[i].size(); ++k) {
          auto v = forward_trajs_[i][k];
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << v.state().time_stamp
                     << ", x: " << v.state().vec_position.x()
                     << ", y: " << v.state().vec_position.y()
                     << ", v: " << v.state().velocity;
        }
        LOG(ERROR) << "[Ssc]ref lane range: [" << nav_lane_local_.begin()
                   << ", " << nav_lane_local_.end() << "]";

        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd velocity (" << start_constraints[1](0)
                   << ", " << start_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd acceleration (" << start_constraints[2](0)
                   << ", " << start_constraints[2](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd position (" << end_constraints[0](0) << ", "
                   << end_constraints[0](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd velocity (" << end_constraints[1](0) << ", "
                   << end_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End state stamp: "
                   << fs_vehicle_traj[num_states - 1].frenet_state.time_stamp;
      }
      bezier_spline_gen_success = false;
    }

    // 低速模式不依赖 Bezier 求解结果，改用 FrenetPrimitive 连接初末状态。
    FrenetPrimitive primitive;
    if (!is_lateral_independent_) {
      primitive.Connect(initial_frenet_state_,
                        fs_vehicle_traj.back().frenet_state,
                        initial_frenet_state_.time_stamp,
                        fs_vehicle_traj.back().frenet_state.time_stamp -
                            initial_frenet_state_.time_stamp,
                        is_lateral_independent_);
    }

    // 正常速度候选的 Bezier 失败则丢弃；低速候选仍进入有效列表。
    if (is_lateral_independent_ && !bezier_spline_gen_success) continue;
    // 历史调试输出保留为关闭状态。
    // printf("[SscQP]spline begin stamp: %lf.\n", bezier_spline.begin());
    // 五个输出容器同步追加，维持相同候选下标。
    qp_trajs_.push_back(bezier_spline);
    primitive_trajs_.push_back(primitive);
    corridors_.push_back(cube_list[i]);
    ref_states_list_.push_back(ref_states);
    valid_behaviors_.push_back(forward_behaviors_[i]);
  }
  return kSuccess;
}

ErrorType SscPlanner::UpdateTrajectoryWithCurrentBehavior() {
  // 至少需要一个完成走廊/轨迹生成的候选。
  int num_valid_behaviors = static_cast<int>(valid_behaviors_.size());
  if (num_valid_behaviors < 1) {
    return kWrongStatus;
  }
  bool find_exact_match_behavior = false;
  int index = 0;
  // 优先查找与行为层最终横向决策完全相同的候选；重复项取最后一个。
  for (int i = 0; i < num_valid_behaviors; i++) {
    if (valid_behaviors_[i] == ego_behavior_) {
      find_exact_match_behavior = true;
      index = i;
    }
  }
  bool find_candidate_behavior = false;
  LateralBehavior candidate_bahavior = common::LateralBehavior::kLaneKeeping;
  if (!find_exact_match_behavior) {
    // 精确行为不可用时，唯一降级策略是寻找车道保持候选。
    for (int i = 0; i < num_valid_behaviors; i++) {
      if (valid_behaviors_[i] == candidate_bahavior) {
        find_candidate_behavior = true;
        index = i;
      }
    }
  }
  if (!find_exact_match_behavior && !find_candidate_behavior)
    return kWrongStatus;

  // 同一候选同时保存两类轨迹；trajectory() 根据速度模式返回其中之一。
  trajectory_ = FrenetBezierTrajectory(qp_trajs_[index], stf_);
  low_spd_alternative_traj_ =
      FrenetPrimitiveTrajectory(primitive_trajs_[index], stf_);
  final_corridor_ = corridors_[index];
  final_ref_states_ = ref_states_list_[index];
  return kSuccess;
}

ErrorType SscPlanner::CorridorFeasibilityCheck(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes) {
  // 空走廊没有可供 spline 分段的时间区间。
  int num_cubes = static_cast<int>(cubes.size());
  if (num_cubes < 1) {
    LOG(ERROR) << "[Ssc]number of cubes not enough.";
    return kWrongStatus;
  }
  // 要求每个前一 cube 的 t 上界与后一 cube 的 t 下界精确相等。
  for (int i = 1; i < num_cubes; i++) {
    if (cubes[i - 1].t_ub != cubes[i].t_lb) {
      LOG(ERROR) << "[Ssc]Err- Corridor not consist.";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i - 1].t_lb << ", "
                 << cubes[i - 1].t_ub << "], x: [" << cubes[i - 1].p_lb[0]
                 << ", " << cubes[i - 1].p_ub[0] << "], y: ["
                 << cubes[i - 1].p_lb[1] << ", " << cubes[i - 1].p_ub[1] << "]";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i].t_lb << ", " << cubes[i].t_ub
                 << "], x: [" << cubes[i].p_lb[0] << ", " << cubes[i].p_ub[0]
                 << "], y: [" << cubes[i].p_lb[1] << ", " << cubes[i].p_ub[1]
                 << "]";
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::StateTransformForInputData() {
  // 使用扁平数组减少重复 StateTransformer 调用和容器分配。
  vec_E<State> global_state_vec;
  vec_E<Vec2f> global_point_vec;
  int num_v;

  // 第一阶段：按固定顺序打包状态和几何点。
  // 先打包规划起始自车状态及其车身顶点，并记录统一顶点数 num_v。
  {
    global_state_vec.push_back(initial_state_);
    vec_E<Vec2f> v_vec;
    common::SemanticsUtils::GetVehicleVertices(ego_vehicle_.param(),
                                               initial_state_, &v_vec);
    num_v = v_vec.size();
    global_point_vec.insert(global_point_vec.end(), v_vec.begin(), v_vec.end());
  }

  // 依次打包每个候选行为的自车 rollout 状态及按自车参数计算的车身顶点。
  {
    common::VehicleParam ego_param = ego_vehicle_.param();
    for (int i = 0; i < (int)forward_trajs_.size(); ++i) {
      if (forward_trajs_[i].size() < 1) continue;
      for (int k = 0; k < (int)forward_trajs_[i].size(); ++k) {
        // 先追加状态。
        State traj_state = forward_trajs_[i][k].state();
        global_state_vec.push_back(traj_state);
        // 再追加与该状态对应的全部车身顶点。
        vec_E<Vec2f> v_vec;
        common::SemanticsUtils::GetVehicleVertices(ego_param, traj_state,
                                                   &v_vec);
        global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                v_vec.end());
      }
    }
  }

  // 按候选、周车 ID、时间顺序打包行为层闭环 rollout。
  {
    for (int i = 0; i < surround_forward_trajs_.size(); ++i) {
      for (auto it = surround_forward_trajs_[i].begin();
           it != surround_forward_trajs_[i].end(); ++it) {
        for (int k = 0; k < it->second.size(); ++k) {
          // 周车状态使用各自 VehicleParam 计算车身顶点。
          State traj_state = it->second[k].state();
          global_state_vec.push_back(traj_state);
          vec_E<Vec2f> v_vec;
          common::SemanticsUtils::GetVehicleVertices(it->second[k].param(),
                                                     traj_state, &v_vec);
          global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                  v_vec.end());
        }
      }
    }
  }

  // 静态障碍栅格只需要二维点转换，不对应 global_state_vec 项。
  {
    for (auto it = obstacle_grids_.begin(); it != obstacle_grids_.end(); ++it) {
      Vec2f pt((*it)[0], (*it)[1]);
      global_point_vec.push_back(pt);
    }
  }

  vec_E<FrenetState> frenet_state_vec(global_state_vec.size());
  vec_E<Vec2f> fs_point_vec(global_point_vec.size());

  // 第二阶段：按编译期开关选择四线程 OpenMP 或单线程批量转换。
#if USE_OPENMP
  TicToc timer_stf;
  StateTransformUsingOpenMp(global_state_vec, global_point_vec,
                            &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]OpenMp transform time cost: " << timer_stf.toc()
               << " ms.";
#else
  TicToc timer_stf;
  StateTransformSingleThread(global_state_vec, global_point_vec,
                             &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]Single thread transform time cost: " << timer_stf.toc()
               << " ms.";
#endif

  // 第三阶段：严格按打包顺序和状态 offset 恢复原有层级容器。
  int offset = 0;
  // 第 0 个状态及前 num_v 个点属于规划起始自车。
  {
    fs_ego_vehicle_.frenet_state = frenet_state_vec[offset];
    fs_ego_vehicle_.vertices.clear();
    for (int i = 0; i < num_v; ++i) {
      fs_ego_vehicle_.vertices.push_back(fs_point_vec[offset * num_v + i]);
    }
    offset++;
  }

  // 重建每个候选行为的自车 Frenet rollout。
  {
    forward_trajs_fs_.clear();
    if (forward_trajs_.size() < 1) return kWrongStatus;
    for (int j = 0; j < (int)forward_trajs_.size(); ++j) {
      // Debug 构建中空候选直接断言；Release 下仍会追加空轨迹。
      if (forward_trajs_[j].size() < 1) assert(false);
      vec_E<common::FsVehicle> traj_fs;
      for (int k = 0; k < (int)forward_trajs_[j].size(); ++k) {
        common::FsVehicle fs_v;
        fs_v.frenet_state = frenet_state_vec[offset];
        // 每个状态假定恰有 num_v 个连续顶点。
        for (int i = 0; i < num_v; ++i) {
          fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
        }
        traj_fs.emplace_back(fs_v);
        offset++;
      }
      forward_trajs_fs_.emplace_back(traj_fs);
    }
  }

  // 按原候选和周车 ID 结构重建周车 Frenet rollout。
  {
    surround_forward_trajs_fs_.clear();
    for (int j = 0; j < surround_forward_trajs_.size(); ++j) {
      std::unordered_map<int, vec_E<common::FsVehicle>> sur_trajs;
      for (auto it = surround_forward_trajs_[j].begin();
           it != surround_forward_trajs_[j].end(); ++it) {
        int v_id = it->first;
        vec_E<common::FsVehicle> traj_fs;
        for (int k = 0; k < it->second.size(); ++k) {
          common::FsVehicle fs_v;
          fs_v.frenet_state = frenet_state_vec[offset];
          // 同样以自车记录的 num_v 作为所有周车顶点跨度。
          for (int i = 0; i < num_v; ++i) {
            fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
          }
          traj_fs.emplace_back(fs_v);
          offset++;
        }
        sur_trajs.insert(
            std::pair<int, vec_E<common::FsVehicle>>(v_id, traj_fs));
      }
      surround_forward_trajs_fs_.emplace_back(sur_trajs);
    }
  }

  // 所有动态状态之后的点区间属于静态障碍栅格。
  {
    obstacle_grids_fs_.clear();
    for (int i = 0; i < static_cast<int>(obstacle_grids_.size()); ++i) {
      // offset 是状态数，每个状态按 num_v 个点换算到点数组下标。
      obstacle_grids_fs_.push_back(fs_point_vec[offset * num_v + i]);
    }
  }

  // 对外 ego_frenet_state_ 与本轮实际规划起始状态保持一致。
  ego_frenet_state_ = fs_ego_vehicle_.frenet_state;
  return kSuccess;
}

ErrorType SscPlanner::StateTransformUsingOpenMp(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  // 输出容器已由调用者按输入大小预分配，这里直接写 data 指针的独立下标。
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();

  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();

  LOG(WARNING) << "[Ssc]OpenMp - Total number of queries: "
               << state_num + point_num;
  // 固定四线程并修改 OpenMP 进程级线程数设置。
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      if (kSuccess != stf_.GetFrenetStateFromState(global_state_vec[i], &fs)) {
        // 状态转换失败时只保留原时间戳，其余 Frenet 字段沿用默认值。
        fs.time_stamp = global_state_vec[i].time_stamp;
      }
      *(ptr_state_vec + i) = fs;
    }
  }
  {
#pragma omp parallel for
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      // 点转换错误码不参与本函数返回状态。
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::StateTransformSingleThread(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  // 单线程路径与 OpenMP 路径保持相同输出布局。
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();
  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();
  {
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      // 当前实现忽略转换错误并写入局部 fs。
      stf_.GetFrenetStateFromState(global_state_vec[i], &fs);
      *(ptr_state_vec + i) = fs;
    }
  }
  {
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      // 当前实现同样忽略点投影错误。
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::set_map_interface(SscPlannerMapItf* map_itf) {
  // 拒绝空接口，但不取得所有权，也不校验接口自身 IsValid。
  if (map_itf == nullptr) return kIllegalInput;
  map_itf_ = map_itf;
  map_valid_ = true;
  return kSuccess;
}

ErrorType SscPlanner::ValidateTrajectory(const FrenetTrajectory& traj) {
  // 在轨迹闭区间内按 0.1 s 生成检查时间序列。
  std::vector<decimal_t> t_vec_xy;
  common::GetRangeVector<decimal_t>(traj.begin(), traj.end(), 0.1, true,
                                    &t_vec_xy);
  common::State state;
  // 检查轨迹起点可求值且笛卡尔位置与规划起点误差不超过 0.1 m。
  if (traj.GetState(traj.begin(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]State evaluation error";
    return kWrongStatus;
  }

  if ((state.vec_position - initial_state_.vec_position).norm() > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init position miss match";
    return kWrongStatus;
  }

  // 起点纵向速度误差不超过 0.1 m/s。
  if (fabs(state.velocity - initial_state_.velocity) > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init vel miss match";
    return kWrongStatus;
  }
  // 终点只要求能够求值，不与参考终端状态比较。
  if (traj.GetState(traj.end(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]End state eval error";
    return kWrongStatus;
  }

  // 全时域唯一的动力学筛选是绝对曲率不超过 0.33 1/m。
  for (const auto t : t_vec_xy) {
    if (traj.GetState(t, &state) != kSuccess) {
      LOG(ERROR) << "[Ssc][Validate]State eval error";
      return kWrongStatus;
    }
    if (fabs(state.curvature) > 0.33) {
      LOG(ERROR) << "[Ssc][Validate]initial_state velocity "
                 << initial_state_.velocity << " Curvature " << state.curvature
                 << " invalid.";
      return kWrongStatus;
    }
  }
  return kSuccess;
}

}  // namespace planning
