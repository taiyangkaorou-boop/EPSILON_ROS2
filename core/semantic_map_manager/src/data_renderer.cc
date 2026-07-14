#include "semantic_map_manager/data_renderer.h"
#include "roguelike_ray_casting/roguelike_ray_casting.h"

#include <algorithm>
#include <iterator>
#include <random>

namespace semantic_map_manager {

DataRenderer::DataRenderer(SemanticMapManager *smm_ptr)
    : p_semantic_map_manager_(smm_ptr) {
  // 从绑定 SMM 读取 ego ID、局部栅格规格和周车搜索半径。
  ego_id_ = p_semantic_map_manager_->ego_id();
  obstacle_map_info_ =
      p_semantic_map_manager_->agent_config_info().obstacle_map_meta_info;
  surrounding_search_radius_ =
      p_semantic_map_manager_->agent_config_info().surrounding_search_radius;

  // GridMap 两维依次使用 height/width，且两个方向共享同一 resolution。
  std::array<int, 2> map_size = {
      {obstacle_map_info_.height, obstacle_map_info_.width}};
  std::array<decimal_t, 2> map_resl = {
      {obstacle_map_info_.resolution, obstacle_map_info_.resolution}};
  std::array<std::string, 2> map_name = {{"height", "width"}};

  // 工作栅格由裸 new 创建；当前空析构函数不会释放该对象。
  p_obstacle_grid_ =
      new common::GridMapND<ObstacleMapType, 2>(map_size, map_resl, map_name);

  printf("[DataRenderer] Initialization finished\n");
}

ErrorType DataRenderer::Render(const double &time_stamp,
                               const common::LaneNet &lane_net,
                               const common::VehicleSet &vehicle_set,
                               const common::ObstacleSet &obstacle_set) {
  time_stamp_ = time_stamp;
  // 自车必须先更新，后续栅格原点、Lane 查询和周车距离都依赖其状态。
  GetEgoVehicle(vehicle_set);
  // 依次构造局部障碍物图、完整/周边 LaneNet 和半径内周车；返回码均未检查。
  GetObstacleMap(obstacle_set);
  GetWholeLaneNet(lane_net);
  GetSurroundingLaneNet(lane_net);
  GetSurroundingVehicles(vehicle_set);

  // 配置开启时模拟跟踪观测噪声，并把不确定 ID 列表同步给 SMM。
  if (p_semantic_map_manager_->agent_config_info().enable_tracking_noise) {
    InjectObservationNoise();
    p_semantic_map_manager_->set_uncertain_vehicle_ids(uncertain_vehicle_ids_);
  }

  TicToc timer;
  // 用八象限 FOV 光线投射模拟局部传感器可见区域，并融合跨帧障碍记忆。
  FakeMapper();
  // printf("[RayCasting]Time cost: %lf ms\n", timer.toc());

  // 将本帧所有派生数据整体写回绑定的 SemanticMapManager。
  p_semantic_map_manager_->UpdateSemanticMap(
      time_stamp_, ego_vehicle_, whole_lane_net_, surrounding_lane_net_,
      *p_obstacle_grid_, obs_grids_, surrounding_vehicles_);

  return kSuccess;
}

ErrorType DataRenderer::InjectObservationNoise() {
  // baseline 只为 ego 0 模拟噪声；其他 ego 直接返回。
  if (ego_id_ != 0) return kSuccess;
  cnt_random_++;

  // 每累计 10 次 Render 才重新采样并实际修改一次当前帧周车状态。
  if (cnt_random_ == 10) {
    uncertain_vehicle_ids_.clear();
    const decimal_t angle_noise_std = 0.22;
    std::normal_distribution<double> lat_pos_dist(0.0, 0.2);
    std::normal_distribution<double> long_pos_dist(0.0, 0.7);
    std::normal_distribution<double> angle_dist(0.0, angle_noise_std);

    // 收集并随机打乱全部周车 ID，最多选取前三辆作为本帧噪声目标。
    std::vector<int> surrounding_ids;
    for (const auto &v : surrounding_vehicles_.vehicles) {
      surrounding_ids.push_back(v.first);
    }
    std::shuffle(surrounding_ids.begin(), surrounding_ids.end(),
                 random_engine_);

    std::vector<int> sampled_ids;
    for (int i = 0; i < 3 && i < surrounding_ids.size(); i++) {
      sampled_ids.push_back(surrounding_ids[i]);
    }

    // 在每辆被选车辆自身航向坐标系中加入纵向/横向位置噪声和航向噪声。
    for (auto &v : surrounding_vehicles_.vehicles) {
      if (std::find(sampled_ids.begin(), sampled_ids.end(), v.first) ==
          sampled_ids.end())
        continue;

      decimal_t lateral_position_noise = lat_pos_dist(random_engine_);
      decimal_t long_position_noise = long_pos_dist(random_engine_);
      decimal_t angle_noise = angle_dist(random_engine_);

      common::State original_state = v.second.state();
      Vec2f original_position = original_state.vec_position;
      decimal_t angle = original_state.angle;
      Vec2f augmented_position =
          Vec2f(original_position.x() + lateral_position_noise * sin(angle) +
                    long_position_noise * cos(angle),
                original_position.y() - lateral_position_noise * cos(angle) +
                    long_position_noise * sin(angle));
      original_state.vec_position = augmented_position;
      original_state.angle =
          normalize_angle(original_state.angle + angle_noise);

      // 只有航向噪声超过 1.5 倍标准差且类型不是 brokencar 时才标为不确定。
      if (fabs(angle_noise) > 1.5 * angle_noise_std &&
          v.second.type().compare("brokencar") != 0)
        uncertain_vehicle_ids_.push_back(v.first);

      v.second.set_state(original_state);
    }
    cnt_random_ = 0;
  }
  return kSuccess;
}

ErrorType DataRenderer::GetEgoVehicle(const common::VehicleSet &vehicle_set) {
  // 使用 map::at 按 ego_id_ 查找；ID 缺失会抛出异常而不是返回 ErrorType。
  ego_vehicle_ = vehicle_set.vehicles.at(ego_id_);
  ego_param_ = ego_vehicle_.param();
  ego_state_ = ego_vehicle_.state();
  return kSuccess;
}

ErrorType DataRenderer::GetObstacleMap(
    const common::ObstacleSet &obstacle_set) {
  // OccupancyGrid 原点位于左下角；世界 x 向右、y 向上。

  // 用自车位置减半幅地图尺寸得到左下角，并取整到整数世界坐标以增强跨帧一致性。
  decimal_t x = ego_state_.vec_position(0) - obstacle_map_info_.h_metric / 2.0;
  decimal_t y = ego_state_.vec_position(1) - obstacle_map_info_.w_metric / 2.0;
  decimal_t x_r = std::round(x);
  decimal_t y_r = std::round(y);

  // 每帧先把全部栅格重置为 UNKNOWN，再设置新的世界原点。
  p_obstacle_grid_->fill_data(GridMap2D::UNKNOWN);
  std::array<decimal_t, 2> origin = {{x_r, y_r}};
  p_obstacle_grid_->set_origin(origin);

  cv::Mat grid_mat =
      cv::Mat(obstacle_map_info_.height, obstacle_map_info_.width,
              CV_MAKETYPE(cv::DataType<ObstacleMapType>::type, 1),
              p_obstacle_grid_->get_data_ptr());

  // 圆形障碍转换到栅格坐标，半径按 resolution 截断为整数像素并实心填充 OCCUPIED。
  for (const auto &obs : obstacle_set.obs_circle) {
    std::array<decimal_t, 2> center_w = {
        {obs.second.circle.center.x, obs.second.circle.center.y}};
    auto center_coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(center_w);
    cv::Point2i center(center_coord[0], center_coord[1]);
    int radius = obs.second.circle.radius / obstacle_map_info_.resolution;
    cv::circle(grid_mat, center, radius, cv::Scalar(GridMap2D::OCCUPIED), -1);
  }
  // 多边形每个世界顶点转换到栅格坐标后，批量调用 OpenCV fillPoly 实心填充。
  std::vector<std::vector<cv::Point>> polys;
  for (const auto &obs : obstacle_set.obs_polygon) {
    std::vector<cv::Point> poly;
    for (const auto &pt : obs.second.polygon.points) {
      std::array<decimal_t, 2> pt_w = {{pt.x, pt.y}};
      auto coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(pt_w);
      cv::Point2i coord_cv(coord[0], coord[1]);
      poly.push_back(coord_cv);
    }
    polys.push_back(poly);
  }
  cv::fillPoly(grid_mat, polys, cv::Scalar(GridMap2D::OCCUPIED));

  return kSuccess;
}

ErrorType DataRenderer::FakeMapper() {
  // 历史障碍只在以自车为中心、半边高度 80% 的轴对齐方形范围内保留。
  decimal_t dist_thres = obstacle_map_info_.h_metric / 2.0 * 0.8;
  // obs_grids_.clear();
  // 当前帧射线投射会把新观察到的障碍世界坐标并入持久 obs_grids_。
  RayCastingOnObstacleMap();

  decimal_t ego_pos_x = ego_state_.vec_position(0);
  decimal_t ego_pos_y = ego_state_.vec_position(1);
  // 删除 x 或 y 距离超阈值的历史点，其余点写回新栅格为 SCANNED_OCCUPIED。
  for (auto it = obs_grids_.begin(); it != obs_grids_.end();) {
    decimal_t dx = abs((*it)[0] - ego_pos_x);
    decimal_t dy = abs((*it)[1] - ego_pos_y);
    if (dx >= dist_thres || dy >= dist_thres) {
      it = obs_grids_.erase(it);
      continue;
    } else {
      p_obstacle_grid_->SetValueUsingGlobalPosition(
          *it, GridMap2D::SCANNED_OCCUPIED);
      ++it;
    }
  }
  return kSuccess;
}

ErrorType DataRenderer::RayCastingOnObstacleMap() {
  // 以自车几何中心而非状态参考点作为 FOV 光线原点。
  Vec3f ray_casting_origin;
  ego_vehicle_.Ret3DofStateAtGeometryCenter(&ray_casting_origin);

  std::array<decimal_t, 2> origin = {
      {ray_casting_origin(0), ray_casting_origin(1)}};
  auto coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(origin);
  // 最大光线半径固定取 GridMap 第一维尺寸的 3/4。
  int r_idx_max = p_obstacle_grid_->dims_size(0) * 3.0 / 4.0;

  std::vector<uint8_t> render_mat(p_obstacle_grid_->data_size(), 0);  // TODO
  // 八次调用分别栅格化八个象限；每个象限返回其首次遇到的障碍坐标集合。
  for (int i = 0; i < 8; ++i) {
    std::set<std::array<int, 2>> obs_coord_set;
    roguelike_ray_casting::RasterizeFOVOctant(
        coord[0], coord[1], r_idx_max, p_obstacle_grid_->dims_size(0),
        p_obstacle_grid_->dims_size(1), i, p_obstacle_grid_->data_ptr(),
        render_mat.data(), &obs_coord_set);
    for (const auto coord : obs_coord_set) {
      std::array<decimal_t, 2> p_w;
      p_obstacle_grid_->GetGlobalPositionUsingCoordinate(coord, &p_w);
      obs_grids_.insert(p_w);
    }
  }
  // 将射线可见性结果与原障碍物栅格逐元素取最大值，再整体替换工作栅格数据。
  for (int i = 0; i < p_obstacle_grid_->data_size(); ++i) {
    render_mat[i] = std::max(render_mat[i], p_obstacle_grid_->data(i));
  }

  p_obstacle_grid_->set_data(render_mat);

  return kSuccess;
}

ErrorType DataRenderer::GetWholeLaneNet(const common::LaneNet &lane_net) {
  // 保存完整 LaneNet 深拷贝，供 SMM 后续构造语义 Lane 和拓扑查询。
  whole_lane_net_ = lane_net;
  return kSuccess;
}

ErrorType DataRenderer::GetSurroundingLaneNet(const common::LaneNet &lane_net) {
  surrounding_lane_net_.clear();
  // TODO(lu.zhang): 当前使用欧氏点云半径，后续可替换为沿 Lane 的拓扑/弧长距离。

  // 展平全部 LaneRaw 采样点；每个 KD 点同时保存 Lane ID 和原采样下标。
  lane_net_pts_.pts.clear();
  for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
       ++iter) {
    for (int i = 0; i < static_cast<int>(iter->second.lane_points.size());
         ++i) {
      common::PointWithValue<int> p;
      int id = iter->second.id;
      p.pt.x = iter->second.lane_points[i](0);
      p.pt.y = iter->second.lane_points[i](1);
      p.values.push_back(id);
      p.values.push_back(i);
      lane_net_pts_.pts.push_back(p);
    }
  }
  // 每帧从头构造并 build 二维 KD-tree，叶节点参数固定为 10。
  kdtree_lane_net_ = std::make_shared<KdTreeFor2dPointVec>(
      2, lane_net_pts_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree_lane_net_->buildIndex();

  const decimal_t query_pt[2] = {ego_vehicle_.state().vec_position(0),
                                 ego_vehicle_.state().vec_position(1)};
  // nanoflann L2 radiusSearch 使用平方半径；此处对应 2*surrounding_search_radius_。
  const decimal_t search_radius =
      surrounding_search_radius_ * surrounding_search_radius_ * 4;
  std::vector<std::pair<size_t, decimal_t>> ret_matches;
  nanoflann::SearchParams params;
  // const size_t nMatches =
  kdtree_lane_net_->radiusSearch(&query_pt[0], search_radius, ret_matches,
                                 params);

  // KD 命中点按 Lane ID 去重，再从输入 LaneNet 复制完整 LaneRaw 到局部集合。
  std::set<size_t> matched_lane_id_set;
  for (const auto &e : ret_matches) {
    matched_lane_id_set.insert(lane_net_pts_.pts[e.first].values[0]);
  }

  for (const auto id : matched_lane_id_set) {
    surrounding_lane_net_.lane_set.insert(
        std::pair<int, common::LaneRaw>(id, lane_net.lane_set.at(id)));
  }
  return kSuccess;
}

ErrorType DataRenderer::GetSurroundingVehicles(
    const common::VehicleSet &vehicle_set) {
  surrounding_vehicles_.vehicles.clear();

  // 线性遍历全部车辆，按 Vehicle 内部 ID 排除 ego，并用平面欧氏距离严格小于阈值筛选。
  for (const auto &v : vehicle_set.vehicles) {
    if (v.second.id() == ego_id_) continue;
    double dx =
        v.second.state().vec_position(0) - ego_vehicle_.state().vec_position(0);
    double dy =
        v.second.state().vec_position(1) - ego_vehicle_.state().vec_position(1);
    double dist = std::hypot(dx, dy);
    if (dist < surrounding_search_radius_) {
      surrounding_vehicles_.vehicles.insert(v);
    }
  }

  return kSuccess;
}

}  // namespace semantic_map_manager
