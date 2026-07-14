/**
 * @file ssc_map.cc
 * @author HKUST Aerial Robotics Group
 * @brief SSC Frenet 时空占用地图与驾驶走廊实现。
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

#include "ssc_planner/ssc_map.h"

#include <glog/logging.h>

namespace planning {

SscMap::SscMap(const SscMap::Config &config) : config_(config) {
  // 启动时完整打印离散化、动力学边界和膨胀参数。
  config_.Print();

  // 原始占用和膨胀占用使用相同的尺寸、分辨率及 s/d/t 轴定义。
  p_3d_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
  p_3d_inflated_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
}

ErrorType SscMap::ResetSscMap(const common::FrenetState &ini_fs) {
  // 新规划周期先丢弃上一周期的离散走廊与两张占用栅格。
  ClearDrivingCorridor();
  ClearGridMap();

  // 以初始 Frenet 状态的时间和纵向位置重建本周期坐标原点。
  start_time_ = ini_fs.time_stamp;
  UpdateMapOrigin(ini_fs);

  return kSuccess;
}

void SscMap::UpdateMapOrigin(const common::FrenetState &ori_fs) {
  // 保存动力学可达范围及首个语义 cube 校验需要的初始状态。
  initial_fs_ = ori_fs;

  // s 原点向后留出 s_back_len，d 轴以 0 为中心，t 原点使用绝对时间戳。
  std::array<decimal_t, 3> map_origin;
  map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;
  map_origin[1] =
      -1 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;
  map_origin[2] = ori_fs.time_stamp;

  // 两张地图必须共享同一坐标系，才能直接比较/可视化对应栅格。
  p_3d_grid_->set_origin(map_origin);
  p_3d_inflated_grid_->set_origin(map_origin);
}

ErrorType SscMap::GetInitialCubeUsingSeed(
    const Vec3i &seed_0, const Vec3i &seed_1,
    common::AxisAlignedCubeNd<int, 3> *cube) const {
  // 每一维分别取两个 seed 的闭区间最小/最大坐标。
  std::array<int, 3> lb;
  std::array<int, 3> ub;
  lb[0] = std::min(seed_0(0), seed_1(0));
  lb[1] = std::min(seed_0(1), seed_1(1));
  lb[2] = std::min(seed_0(2), seed_1(2));
  ub[0] = std::max(seed_0(0), seed_1(0));
  ub[1] = std::max(seed_0(1), seed_1(1));
  ub[2] = std::max(seed_0(2), seed_1(2));

  *cube = common::AxisAlignedCubeNd<int, 3>(ub, lb);
  return kSuccess;
}

ErrorType SscMap::ConstructSscMap(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs,
    const vec_E<Vec2f> &obstacle_grids) {
  // 每次构建都从全空地图开始，避免上一周期或上一候选的占用残留。
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  // 静态点沿时间轴拉伸，动态周车按预测时间层写入多边形占用。
  FillStaticPart(obstacle_grids);
  FillDynamicPart(sur_vehicle_trajs_fs);
  return kSuccess;
}

ErrorType SscMap::GetInflationDirections(const bool &if_first_cube,
                                         std::array<bool, 6> *dirs_disabled) {
  // s/d 双向和 +t 默认允许；仅首 cube 设计上允许向 -t 扩张。
  (*dirs_disabled)[0] = false;
  (*dirs_disabled)[1] = false;
  (*dirs_disabled)[2] = false;
  (*dirs_disabled)[3] = false;
  (*dirs_disabled)[4] = false;
  (*dirs_disabled)[5] = !if_first_cube;

  return kSuccess;
}

ErrorType SscMap::ClearGridMap() {
  // GridMapND::clear_data 将所有栅格恢复为数值零。
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  return kSuccess;
}

ErrorType SscMap::ClearDrivingCorridor() {
  // 当前函数只清离散走廊，不联动清理最终连续 corridor 缓存。
  driving_corridor_vec_.clear();
  return kSuccess;
}

ErrorType SscMap::ConstructCorridorUsingInitialTrajectory(
    GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs) {
  // 第一阶段：把初始 Frenet 状态和行为层轨迹采样转换为三维整数 seed。
  vec_E<Vec3i> traj_seeds;
  int num_states = static_cast<int>(trajs.size());
  if (num_states > 1) {
    bool first_seed_determined = false;
    for (int k = 0; k < num_states; ++k) {
      std::array<decimal_t, 3> p_w = {};
      if (!first_seed_determined) {
        // 首个 seed 固定取本周期 initial_fs_，第二个取首个合法未来轨迹状态。
        decimal_t s_0 = initial_fs_.vec_s[0];
        decimal_t d_0 = initial_fs_.vec_dt[0];
        decimal_t t_0 = initial_fs_.time_stamp;
        std::array<decimal_t, 3> p_w_0 = {s_0, d_0, t_0};
        auto coord_0 = p_grid->GetCoordUsingGlobalPosition(p_w_0);

        decimal_t s_1 = trajs[k].frenet_state.vec_s[0];
        decimal_t d_1 = trajs[k].frenet_state.vec_dt[0];
        decimal_t t_1 = trajs[k].frenet_state.time_stamp;
        std::array<decimal_t, 3> p_w_1 = {s_1, d_1, t_1};
        auto coord_1 = p_grid->GetCoordUsingGlobalPosition(p_w_1);
        // 丢弃地图 s/d/t 范围外的轨迹状态。
        if (!p_grid->CheckCoordInRange(coord_1)) {
          continue;
        }
        // t 下标 0 及以前不作为未来 seed。
        if (coord_1[2] <= 0) {
          continue;
        }

        first_seed_determined = true;
        traj_seeds.push_back(Vec3i(coord_0[0], coord_0[1], coord_0[2]));
        traj_seeds.push_back(Vec3i(coord_1[0], coord_1[1], coord_1[2]));
      } else {
        // 首对 seed 建立后，其余合法状态直接按 s/d/t 坐标追加。
        decimal_t s = trajs[k].frenet_state.vec_s[0];
        decimal_t d = trajs[k].frenet_state.vec_dt[0];
        decimal_t t = trajs[k].frenet_state.time_stamp;
        p_w = {s, d, t};
        auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);
        // 超出当前有限时空窗口的后续采样直接跳过。
        if (!p_grid->CheckCoordInRange(coord)) {
          continue;
        }
        traj_seeds.push_back(Vec3i(coord[0], coord[1], coord[2]));
      }
    }
  }

  // 第二阶段：沿 seed 序列构造初始 cube，并在空闲空间内逐轴膨胀。
  common::DrivingCorridor driving_corridor;
  bool is_valid = true;
  auto seed_num = static_cast<int>(traj_seeds.size());
  if (seed_num < 2) {
    // 无法形成至少一个 seed 区间时仍保存一个 invalid corridor 供候选对齐。
    driving_corridor.is_valid = false;
    driving_corridor_vec_.push_back(driving_corridor);
    is_valid = false;
    return kWrongStatus;
  }
  for (int i = 0; i < seed_num; ++i) {
    if (i == 0) {
      // 首 cube 以 seed[0]—seed[1] 的最小包围盒初始化。
      common::AxisAlignedCubeNd<int, 3> cube;
      GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);
      if (!CheckIfCubeIsFree(p_grid, cube)) {
        // 初始包围盒已占用时记录冲突 cube/seeds，并终止该候选走廊。
        LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;

        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_cube.seeds.push_back(traj_seeds[i + 1]);
        driving_corridor.cubes.push_back(driving_cube);

        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        break;
      }

      std::array<bool, 6> dirs_disabled = {false, false, false,
                                           false, false, false};
      // 当前主流程允许全部方向标志，但实际膨胀器只执行 s/d 双向和 +t。
      InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps, &cube);

      common::DrivingCube driving_cube;
      driving_cube.cube = cube;
      driving_cube.seeds.push_back(traj_seeds[i]);
      driving_corridor.cubes.push_back(driving_cube);
    } else {
      // 新 seed 仍位于当前膨胀 cube 内时，仅追加到该 cube 的 seed 序列。
      if (CheckIfCubeContainsSeed(driving_corridor.cubes.back().cube,
                                  traj_seeds[i])) {
        driving_corridor.cubes.back().seeds.push_back(traj_seeds[i]);
        continue;
      } else {
        // 新 seed 离开当前 cube：取最后一个内部 seed 作为两个 cube 的连接点。
        Vec3i seed_r = driving_corridor.cubes.back().seeds.back();
        driving_corridor.cubes.back().seeds.pop_back();
        // 将上一 cube 的时间上界裁到连接 seed，并回退 i 构造下一 seed 区间。
        driving_corridor.cubes.back().cube.upper_bound[2] = seed_r(2);
        i = i - 1;

        common::AxisAlignedCubeNd<int, 3> cube;
        GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);

        if (!CheckIfCubeIsFree(p_grid, cube)) {
          // 新区间碰撞时保留冲突 cube，标记整个候选走廊无效并结束。
          LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: "
                     << i;
          common::DrivingCube driving_cube;
          driving_cube.cube = cube;
          driving_cube.seeds.push_back(traj_seeds[i]);
          driving_cube.seeds.push_back(traj_seeds[i + 1]);
          driving_corridor.cubes.push_back(driving_cube);

          driving_corridor.is_valid = false;
          driving_corridor_vec_.push_back(driving_corridor);
          is_valid = false;
          break;
        }

        std::array<bool, 6> dirs_disabled = {false, false, false,
                                             false, false, false};
        // 对新区间重复相同的受占用/动力学边界膨胀。
        InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps,
                            &cube);
        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_corridor.cubes.push_back(driving_cube);
      }
    }
  }
  if (is_valid) {
    // 走廊相邻 cube 的额外交叠松弛当前被关闭。
    // CorridorRelaxation(p_grid, &driving_corridor);
    // 最后一个 cube 的时间上界裁到最后一个有效 seed，避免超出初始轨迹时域。
    driving_corridor.cubes.back().cube.upper_bound[2] = traj_seeds.back()(2);
    driving_corridor.is_valid = true;
    driving_corridor_vec_.push_back(driving_corridor);
  }

  return kSuccess;
}

ErrorType SscMap::GetTimeCoveredCubeIndices(
    const common::DrivingCorridor *p_corridor, const int &start_idx,
    const int &dir, const int &t_trans, std::vector<int> *idx_list) const {
  // 从 start_idx 沿前/后方向累计各 cube 的离散时间跨度。
  int dt = 0;
  int num_cube = p_corridor->cubes.size();
  int idx = start_idx;
  while (idx < num_cube && idx >= 0) {
    dt += p_corridor->cubes[idx].cube.upper_bound[2] -
          p_corridor->cubes[idx].cube.lower_bound[2];
    idx_list->push_back(idx);
    // dir==1 向未来遍历，其余值均按向过去遍历处理。
    if (dir == 1) {
      ++idx;
    } else {
      --idx;
    }
    if (dt >= t_trans) {
      break;
    }
  }
  return kSuccess;
}

ErrorType SscMap::CorridorRelaxation(GridMap3D *p_grid,
                                     common::DrivingCorridor *p_corridor) {
  // s/d 方向期望交叠裕量分别为 50/10 个栅格，传播时间跨度固定为 7 个栅格。
  std::array<int, 2> margin = {{50, 10}};
  int t_trans = 7;
  int num_cube = p_corridor->cubes.size();
  for (int i = 0; i < num_cube - 1; ++i) {
    if (1)  // 启用 s 方向相邻走廊松弛。
    {
      // 比较相邻 cube 的 s 上下界；间距小于目标裕量时向连接处双向扩张。
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[0];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[0];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[0];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[0];

      if (abs(cube_0_ub - cube_1_lb) < margin[0]) {
        int room = margin[0] - abs(cube_0_ub - cube_1_lb);
        // 向未来收集覆盖固定时长的 cube。
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // 向过去收集覆盖固定时长的 cube。
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[0]) {
        int room = margin[0] - abs(cube_0_lb - cube_1_ub);
        // 对相反的 s 边界关系执行镜像扩张。
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }

    if (1)  // 启用 d 方向相邻走廊松弛。
    {
      // d 方向采用相同策略，但目标交叠裕量较小。
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[1];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[1];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[1];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[1];

      if (abs(cube_0_ub - cube_1_lb) < margin[1]) {
        int room = margin[1] - abs(cube_0_ub - cube_1_lb);
        // 将扩张传播到连接点未来/过去固定时间范围内的多个 cube。
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[1]) {
        int room = margin[1] - abs(cube_0_lb - cube_1_ub);
        // 相反 d 边界关系按镜像方向扩张。
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto idx : up_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto idx : down_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::InflateObstacleGrid(const common::VehicleParam &param) {
  // 根据车辆参考点到车头/车尾的距离，计算 s 正负方向占用膨胀长度。
  decimal_t s_p_inflate_len = param.length() / 2.0 - param.d_cr();
  decimal_t s_n_inflate_len = param.length() - s_p_inflate_len;
  int num_s_p_inflate_grids =
      std::floor(s_p_inflate_len / config_.map_resolution[0]);
  int num_s_n_inflate_grids =
      std::floor(s_n_inflate_len / config_.map_resolution[0]);
  // 横向宽度额外减去固定 0.5 m 后，再按两侧平均换算为栅格数。
  int num_d_inflate_grids =
      std::floor((param.width() - 0.5) / 2.0 / config_.map_resolution[1]);
  bool is_free = false;

  // 全量扫描原始 s/d/t 栅格；每个非零单元向车辆足迹范围写入膨胀地图。
  for (int i = 0; i < config_.map_size[0]; ++i) {
    for (int j = 0; j < config_.map_size[1]; ++j) {
      for (int k = 0; k < config_.map_size[2]; ++k) {
        std::array<int, 3> coord = {i, j, k};
        p_3d_grid_->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          // 循环上界为开区间，写越界行为由 GridMapND 内部返回码处理。
          for (int s = -num_s_n_inflate_grids; s < num_s_p_inflate_grids; s++) {
            for (int d = -num_d_inflate_grids; d < num_d_inflate_grids; d++) {
              coord = {i + s, j + d, k};
              p_3d_inflated_grid_->SetValueUsingCoordinate(coord, 100);
            }
          }
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::InflateCubeIn3dGrid(GridMap3D *p_grid,
                                      const std::array<bool, 6> &dir_disabled,
                                      const std::array<int, 6> &dir_step,
                                       common::AxisAlignedCubeNd<int, 3> *cube) {
  // 六方向顺序为 +s、-s、+d、-d、+t、-t；当前函数不执行 -t 膨胀。
  bool x_p_finish = dir_disabled[0];
  bool x_n_finish = dir_disabled[1];
  bool y_p_finish = dir_disabled[2];
  bool y_n_finish = dir_disabled[3];
  bool z_p_finish = dir_disabled[4];

  int x_p_step = dir_step[0];
  int x_n_step = dir_step[1];
  int y_p_step = dir_step[2];
  int y_n_step = dir_step[3];
  int z_p_step = dir_step[4];

  // 以 cube 下界时间为起点，限制单 cube 向未来覆盖的最大离散时间跨度。
  int t_max_grids = cube->lower_bound[2] + config_.kMaxNumOfGridAlongTime;

  // 用初始纵向速度和全局加/减速度界估计该时间处的可达 s 上下界。
  decimal_t t = t_max_grids * p_grid->dims_resolution(2);
  decimal_t a_max = config_.kMaxLongitudinalAcc;
  decimal_t a_min = config_.kMaxLongitudinalDecel;
  // 额外以前一秒匀速位移量作为上下界补偿。
  decimal_t d_comp = initial_fs_.vec_s[1] * 1;

  decimal_t s_u = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_max * t * t + d_comp;
  decimal_t s_l = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_min * t * t - d_comp;

  int s_idx_u, s_idx_l;
  // 将连续可达边界映射到 s 栅格，并限制后向扩张不超过预设后方区域的一半。
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_u, 0, &s_idx_u);
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_l, 0, &s_idx_l);
  s_idx_l = std::max(s_idx_l, static_cast<int>((config_.s_back_len / 2.0) /
                                               config_.map_resolution[0]));

  // s/d 四个方向轮流按批次扩张，直到全部遇到障碍、地图边界或动力学边界。
  while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {
    if (!x_p_finish) x_p_finish = InflateCubeOnXPosAxis(p_grid, x_p_step, cube);
    if (!x_n_finish) x_n_finish = InflateCubeOnXNegAxis(p_grid, x_n_step, cube);

    if (!y_p_finish) y_p_finish = InflateCubeOnYPosAxis(p_grid, y_p_step, cube);
    if (!y_n_finish) y_n_finish = InflateCubeOnYNegAxis(p_grid, y_n_step, cube);

    if (cube->upper_bound[0] >= s_idx_u) x_p_finish = true;
    if (cube->lower_bound[0] <= s_idx_l) x_n_finish = true;
  }

  // 时间轴只向未来扩张，不向过去扩张。
  while (!z_p_finish) {
    if (!z_p_finish) z_p_finish = InflateCubeOnZPosAxis(p_grid, z_p_step, cube);

    if (cube->upper_bound[2] - cube->lower_bound[2] >=
        config_.kMaxNumOfGridAlongTime) {
      z_p_finish = true;
    }
  }

  return kSuccess;
}

bool SscMap::InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  // 每次尝试把 +s 边界向外移动一层，最多连续移动 n_step 层。
  for (int i = 0; i < n_step; ++i) {
    int x = cube->upper_bound[0] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        // 新增 s 截面全部空闲，提交新的上界。
        cube->upper_bound[0] = x;
      } else {
        // 新增截面含占用，保持旧边界并结束该方向。
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  // -s 方向与 +s 对称，越界或遇到占用时返回完成。
  for (int i = 0; i < n_step; ++i) {
    int x = cube->lower_bound[0] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        // 新增 s 截面全部空闲，提交新的下界。
        cube->lower_bound[0] = x;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  // 沿 +d 方向逐层检查覆盖当前 s/t 范围的新截面。
  for (int i = 0; i < n_step; ++i) {
    int y = cube->upper_bound[1] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        // 新增 d 截面空闲，提交新的上界。
        cube->upper_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  // 沿 -d 方向逐层检查，逻辑与 +d 镜像。
  for (int i = 0; i < n_step; ++i) {
    int y = cube->lower_bound[1] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        // 新增 d 截面空闲，提交新的下界。
        cube->lower_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  // 沿 +t 方向逐层检查覆盖当前 s/d 范围的新时间截面。
  for (int i = 0; i < n_step; ++i) {
    int z = cube->upper_bound[2] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        // 新时间层空闲，提交新的时间上界。
        cube->upper_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  // -t 方向辅助函数保留，但当前 InflateCubeIn3dGrid 不调用。
  for (int i = 0; i < n_step; ++i) {
    int z = cube->lower_bound[2] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        // 新时间层空闲，提交新的时间下界。
        cube->lower_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::CheckIfCubeIsFree(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube) const {
  // 提取三个维度的闭区间边界，随后穷举 cube 中每个栅格单元。
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  int f2_min = cube.lower_bound[2];
  int f2_max = cube.upper_bound[2];

  int i, j, k;
  std::array<int, 3> coord;
  bool is_free;
  for (i = f0_min; i <= f0_max; ++i) {
    for (j = f1_min; j <= f1_max; ++j) {
      for (k = f2_min; k <= f2_max; ++k) {
        coord = {i, j, k};
        p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        // 任意单元非零即认为整个 cube 不可用并提前返回。
        if (!is_free) {
          return false;
        }
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnXAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &x) const {
  // 固定 s=x，遍历当前 cube 的完整 d/t 闭区间。
  int f0_min = cube.lower_bound[1];
  int f0_max = cube.upper_bound[1];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {x, i, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnYAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &y) const {
  // 固定 d=y，遍历当前 cube 的完整 s/t 闭区间。
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, y, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnZAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &z) const {
  // 固定 t=z，遍历当前 cube 的完整 s/d 闭区间。
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, j, z};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfCubeContainsSeed(
    const common::AxisAlignedCubeNd<int, 3> &cube_a, const Vec3i &seed) const {
  // 三个维度都必须落在含端点的上下界内。
  for (int i = 0; i < 3; ++i) {
    if (cube_a.lower_bound[i] > seed(i) || cube_a.upper_bound[i] < seed(i)) {
      return false;
    }
  }
  return true;
}

ErrorType SscMap::GetFinalGlobalMetricCubesList() {
  // 每次转换都重建最终结果和候选有效标志。
  final_corridor_vec_.clear();
  if_corridor_valid_.clear();
  for (const auto &corridor : driving_corridor_vec_) {
    vec_E<common::SpatioTemporalSemanticCubeNd<2>> cubes;
    if (!corridor.is_valid) {
      // 无效候选仍追加空 cube 列表，以维持与候选行为的下标对应。
      if_corridor_valid_.push_back(0);
    } else {
      if_corridor_valid_.push_back(1);
      for (int k = 0; k < static_cast<int>(corridor.cubes.size()); ++k) {
        common::SpatioTemporalSemanticCubeNd<2> cube;
        decimal_t x_lb, x_ub;
        decimal_t y_lb, y_ub;
        decimal_t z_lb, z_ub;

        // 将离散 s/d/t 上下界坐标分别转换为 GridMap 世界指标。
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[0], 0, &x_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[0], 0, &x_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[1], 1, &y_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[1], 1, &y_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[2], 2, &z_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[2], 2, &z_ub);

        cube.t_lb = z_lb;
        cube.t_ub = z_ub;

        // s 位置继承走廊边界，纵向速度/加速度使用 Config 的全局统一约束。
        cube.p_lb[0] = x_lb;
        cube.p_ub[0] = x_ub;
        cube.v_lb[0] = config_.kMinLongitudinalVel;
        cube.v_ub[0] = config_.kMaxLongitudinalVel;
        cube.a_lb[0] = config_.kMaxLongitudinalDecel;
        cube.a_ub[0] = config_.kMaxLongitudinalAcc;

        // d 位置继承走廊边界，横向速度/加速度使用对称统一约束。
        cube.p_lb[1] = y_lb;
        cube.p_ub[1] = y_ub;
        cube.v_lb[1] = -config_.kMaxLateralVel;
        cube.v_ub[1] = config_.kMaxLateralVel;
        cube.a_lb[1] = -config_.kMaxLateralAcc;
        cube.a_ub[1] = config_.kMaxLateralAcc;

        if (k == 0) {
          // 首个连续 cube 必须覆盖 initial_fs_ 的横向位置，否则终止转换。
          if (y_lb > initial_fs_.vec_dt[0] || y_ub < initial_fs_.vec_dt[0]) {
            LOG(ERROR) << "[Ssc] SscMap - Initial state out of bound d: "
                       << initial_fs_.vec_dt[0] << ", lb: " << y_lb
                       << ", ub: " << y_ub;
            // 原始断言被禁用，当前用 kWrongStatus 向上传递。
            // assert(false);
            return kWrongStatus;
          }
        }

        cubes.push_back(cube);
      }
    }
    final_corridor_vec_.push_back(cubes);
  }

  return kSuccess;
}

ErrorType SscMap::FillStaticPart(const vec_E<Vec2f> &obs_grid_fs) {
  // 每个静态 Frenet 障碍点在所有离散时间层占用同一 s/d 单元。
  for (int i = 0; i < static_cast<int>(obs_grid_fs.size()); ++i) {
    // 原始实现忽略纵向坐标不大于零的障碍点。
    if (obs_grid_fs[i](0) <= 0) {
      continue;
    }
    for (int k = 0; k < config_.map_size[2]; ++k) {
      // 时间坐标从 0 开始按分辨率递增，再交给带 origin 的 GridMap 做坐标转换。
      std::array<decimal_t, 3> pt = {{obs_grid_fs[i](0), obs_grid_fs[i](1),
                                      (double)k * config_.map_resolution[2]}};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
      if (p_3d_grid_->CheckCoordInRange(coord)) {
        // 占用值统一写为 100。
        p_3d_grid_->SetValueUsingCoordinate(coord, 100);
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::FillDynamicPart(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs) {
  // unordered_map 的 key 为周车 ID，value 为该车完整 Frenet 预测轨迹。
  for (auto it = sur_vehicle_trajs_fs.begin(); it != sur_vehicle_trajs_fs.end();
       ++it) {
    // 单车填图错误当前不向上传播，仍继续处理其余周车。
    FillMapWithFsVehicleTraj(it->second);
  }
  return kSuccess;
}

ErrorType SscMap::FillMapWithFsVehicleTraj(
    const vec_E<common::FsVehicle> traj) {
  // 空轨迹不能形成任何动态占用。
  if (traj.size() == 0) {
    LOG(ERROR) << "[Ssc] SscMap - Trajectory is empty.";
    return kWrongStatus;
  }
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    // 每个 FsVehicle 状态包含同一时间的车身多边形顶点。
    bool is_valid = true;
    for (const auto &v : traj[i].vertices) {
      // 任一顶点 s<=0 时丢弃该时刻整车占用。
      if (v(0) <= 0) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) {
      continue;
    }
    decimal_t z = traj[i].frenet_state.time_stamp;
    int t_idx = 0;
    std::vector<common::Point2i> v_coord;
    std::array<decimal_t, 3> p_w;
    for (const auto &v : traj[i].vertices) {
      // 将每个 Frenet 顶点的 s/d 和状态时间戳映射为三维栅格坐标。
      p_w = {v(0), v(1), z};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
      t_idx = coord[2];
      if (!p_3d_grid_->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      v_coord.push_back(common::Point2i(coord[0], coord[1]));
    }
    if (!is_valid) {
      continue;
    }
    std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    std::vector<cv::Point2i> v_coord_cv;
    // OpenCV fillPoly 需要 cv::Point2i 多边形列表。
    common::ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(v_coord,
                                                             &v_coord_cv);
    vv_coord_cv.push_back(v_coord_cv);
    int w = p_3d_grid_->dims_size()[0];
    int h = p_3d_grid_->dims_size()[1];
    // GridMapND 数据按时间层连续存储，计算目标层首地址并包装为 d×s Mat 视图。
    int layer_offset = t_idx * w * h;
    cv::Mat layer_mat =
        cv::Mat(h, w, CV_MAKETYPE(cv::DataType<SscMapDataType>::type, 1),
                p_3d_grid_->get_data_ptr() + layer_offset);
    // 在该离散时间层把整个车辆多边形内部填为占用值 100。
    cv::fillPoly(layer_mat, vv_coord_cv, 100);
  }
  return kSuccess;
}

}  // namespace planning
