#include "phy_simulator/phy_simulator.h"


namespace phy_simulator {

PhySimulation::PhySimulation() {
  // 空路径加载器会立即进入三个解析函数；该路径主要是遗留默认构造接口。
  p_arena_loader_ = new ArenaLoader();
  if (!GetDataFromArenaLoader()) assert(false);
}

PhySimulation::PhySimulation(const std::string &vehicle_set_path,
                             const std::string &map_path,
                             const std::string &lane_net_path) {
  std::cout << "[PhySimulation] Constructing..." << std::endl;
  // 先创建加载器，再逐项注入三类场景路径。
  p_arena_loader_ = new ArenaLoader();

  p_arena_loader_->set_vehicle_set_path(vehicle_set_path);
  p_arena_loader_->set_map_path(map_path);
  p_arena_loader_->set_lane_net_path(lane_net_path);

  // 加载 common 场景对象后，为每辆车建立独立运动学模型。
  GetDataFromArenaLoader();
  SetupVehicleModelForVehicleSet();
}

bool PhySimulation::GetDataFromArenaLoader() {
  std::cout << "[PhySimulation] Parsing simulation info..." << std::endl;
  // 三个解析返回状态均被忽略，本函数固定返回 true。
  p_arena_loader_->ParseVehicleSet(&vehicle_set_);
  p_arena_loader_->ParseMapInfo(&obstacle_set_);
  p_arena_loader_->ParseLaneNetInfo(&lane_net_);
  return true;
}

bool PhySimulation::AddTemporaryObstacleToMap(const common::Point &pt,
                                              const double &s) {
  // 按给定中心和边长构造四个角点，顶点顺序为逆时针且不重复闭合点。
  common::PolygonObstacle obs;
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y - s / 2));
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y - s / 2));
  obs.type = 1;

  // 临时 ID 从固定 10000 偏移开始单调递增。
  obs.id = temp_obs_idx_offset_ + temp_obstacle_cnt_;
  // 同一对象同时写入环境总集合和临时对象索引。
  obstacle_set_.obs_polygon.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  temp_obstacle_set_.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  ++temp_obstacle_cnt_;
  return true;
}

bool PhySimulation::SetupVehicleModelForVehicleSet() {
  // 当前忽略 Vehicle.type，为所有车辆统一使用五状态运动学自行车模型。
  for (const auto &p : vehicle_set_.vehicles) {
    simulator::VehicleModel vehicle_model(
        p.second.param().wheel_base(), p.second.param().max_steering_angle());
    vehicle_model.set_state(p.second.state());
    // model map 以车辆输入 map key 为 ID；重复 setup 时 insert 不覆盖。
    vehicle_model_set_.insert(
        std::pair<int, simulator::VehicleModel>(p.first, vehicle_model));

    // ID 列表保留 unordered_map 本次遍历顺序。
    vehicle_ids_.push_back(p.first);
  }
  return true;
}

bool PhySimulation::UpdateSimulatorUsingSignalSet(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  // 计时器当前未读取；公开包装也不传播内部更新结果。
  TicToc updata_vehicle_time;
  UpdateVehicleStates(signal_set, dt);
  return true;
}

bool PhySimulation::UpdateVehicleStates(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  // 先要求信号数量与车辆数量相同，但不验证两组 ID 完全一致。
  if (signal_set.signal_set.size() != vehicle_set_.vehicles.size()) {
    std::cerr << "[PhySimulation] ERROR - Signal number error." << std::endl;
    std::cerr << "[PhySimulation] signal_set num: "
              << signal_set.signal_set.size()
              << ", vehicle_set num: " << vehicle_set_.vehicles.size()
              << std::endl;
    assert(false);
  }
  for (auto iter = vehicle_set_.vehicles.begin();
       iter != vehicle_set_.vehicles.end(); ++iter) {
    int id = iter->first;
    // `.at(id)` 要求每辆车均存在同 ID 信号，否则抛出 out_of_range。
    common::VehicleControlSignal signal = signal_set.signal_set.at(id);
    decimal_t steer_rate = signal.steer_rate;
    decimal_t acc = signal.acc;

    auto model_iter = vehicle_model_set_.find(id);

    if (!signal.is_openloop) {
      // 闭环路径把转角速度/纵向加速度作为常值控制推进 dt。
      model_iter->second.set_control(
          simulator::VehicleModel::Control(steer_rate, acc));
      model_iter->second.Step(dt);
    } else {
      // openloop 路径不积分，直接把信号携带的完整 State 写入模型。
      model_iter->second.set_state(signal.state);
    }
    // 无论哪种路径，都把模型最终 State 同步回 VehicleSet。
    iter->second.set_state(model_iter->second.state());
  }
  return true;
}

bool PhySimulation::RemoveTemporaryObstacle(const common::Point &pt,
                                             const double &s) {
  // 遍历临时索引，按多边形顶点算术平均值近似障碍中心。
  for (auto it = temp_obstacle_set_.begin(); it != temp_obstacle_set_.end();) {
    decimal_t sum_x = 0, sum_y = 0;
    for (const auto &p : it->second.polygon.points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    decimal_t center_x = sum_x / it->second.polygon.points.size();
    decimal_t center_y = sum_y / it->second.polygon.points.size();
    decimal_t dx = center_x - pt.x;
    decimal_t dy = center_y - pt.y;
    decimal_t d = std::hypot(dx, dy);

    // 删除半径内所有临时对象，并同步从总 ObstacleSet 中移除相同 ID。
    if (d < s) {
      auto it_obs_set = obstacle_set_.obs_polygon.find(it->first);
      if (it_obs_set != obstacle_set_.obs_polygon.end()) {
        obstacle_set_.obs_polygon.erase(it_obs_set);
      }
      it = temp_obstacle_set_.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

}  // namespace phy_simulator
