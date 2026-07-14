#include "semantic_map_manager/traffic_signal_manager.h"

namespace semantic_map_manager {

// 构造即加载场景信号；baseline 忽略加载返回码。
TrafficSignalManager::TrafficSignalManager() { LoadSignals(); }

// 初始化入口尚未承担额外资源或状态准备职责。
ErrorType TrafficSignalManager::Init() { return kSuccess; }

ErrorType TrafficSignalManager::LoadSignals() {
  // 当前信号来源仍是场景相关的硬编码示例，尚未接入物理仿真或外部地图配置。
  // TODO: (@denny.ding) traffic signal should be controlled by phy sim
  // 以下 google_urban/highway 限速构造均被注释，因此默认列表保持为空。
  {
      // common::SpeedLimit speed_limit1(Vec2f(549.927, 2.587),
      //                                 Vec2f(545.641, -96.123),
      //                                 Vec2f(0.0, 8.0));
      // speed_limit1.set_lateral_range(Vec2f(-1.75, 5.25));
      // speed_limit1.set_start_angle(1.530);
      // speed_limit1.set_end_angle(1.616);
      // speed_limit_list_.push_back(speed_limit1);
  } {
      // common::SpeedLimit speed_limit1(Vec2f(552.916, -218.287),
      //                                 Vec2f(541.683, -238.290),
      //                                 Vec2f(0.0, 6.0));
      // speed_limit1.set_start_angle(1.610);
      // speed_limit1.set_end_angle(0.0);
      // speed_limit1.set_lateral_range(Vec2f(-1.75, 5.25));
      // speed_limit_list_.push_back(speed_limit1);
  } {
    // ~ for left turn
    // common::SpeedLimit speed_limit1(Vec2f(532.304, -257.094),
    //                                 Vec2f(572.222, -223.729),
    //                                 Vec2f(0.0, 5.0));
    // speed_limit1.set_lateral_range(Vec2f(-20, 20));
    // speed_limit1.set_start_angle(3.046);
    // speed_limit1.set_end_angle(-1.571);
    // speed_limit_list_.push_back(speed_limit1);
  }  // {
  // common::SpeedLimit speed_limit1(Vec2f(547.783, -219.819),
  //                                 Vec2f(530.753, -232.968), Vec2f(0.0, 5.0));
  // speed_limit1.set_lateral_range(Vec2f(-20, 20));
  // speed_limit1.set_start_angle(1.603);
  // speed_limit1.set_end_angle(-0.042);
  // speed_limit_list_.push_back(speed_limit1);
  // }
  // {
  //   common::SpeedLimit speed_limit1(Vec2f(501.245, -235.298),
  //                                   Vec2f(489.041, -234.299), Vec2f(0.0,
  //                                   0.5));
  //   speed_limit1.set_lateral_range(Vec2f(-1.75, 5.25));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  // * For highway
  // {
  //   Benchmark
  //   common::SpeedLimit speed_limit1(Vec2f(193.181, -396.072),
  //                                   Vec2f(227.831, -465.545),
  //                                   Vec2f(0.0, 4.0));
  //   speed_limit1.set_start_angle(2.034);
  //   speed_limit1.set_end_angle(2.034);
  //   speed_limit1.set_lateral_range(Vec2f(-10, 10));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  // {
  //   Benchmark
  //   common::SpeedLimit speed_limit1(Vec2f(153.855, -315.488),
  //                                   Vec2f(161.858, -332.227), Vec2f(0.0,
  //                                   0.0));
  //   speed_limit1.set_start_angle(2.034);
  //   speed_limit1.set_end_angle(2.034);
  //   speed_limit1.set_lateral_range(Vec2f(-10, 10));
  //   speed_limit1.set_valid_time(Vec2f(0.0, 65.0));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  // {
  //   common::SpeedLimit speed_limit1(Vec2f(615.498, -1137.748),
  //                                   Vec2f(682.836, -1359.883),
  //                                   Vec2f(0.0, 15.0));
  //   speed_limit1.set_lateral_range(Vec2f(-10, 10));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  return kSuccess;
}

ErrorType TrafficSignalManager::UpdateSignals(const decimal_t time_elapsed) {
  // printf("[xx]time elapsed: %lf.\n", time_elapsed);
  // 直接从容器中永久移除当前时刻早于起点或晚于终点的限速信号。
  for (auto it = speed_limit_list_.begin(); it < speed_limit_list_.end();) {
    Vec2f valid_time = it->valid_time();
    if (time_elapsed < valid_time[0] || time_elapsed > valid_time[1]) {
      printf("[xxxxx]signal erased at %lf.\n", time_elapsed);
      it = speed_limit_list_.erase(it);
    } else {
      ++it;
    }
  }
  return kSuccess;
}

ErrorType TrafficSignalManager::GetSpeedLimit(const State& state,
                                              const Lane& lane,
                                              decimal_t* speed_limit) const {
  // 先把查询状态投影到参考 Lane；投影失败时不写输出并返回错误。
  common::StateTransformer stf(lane);
  common::FrenetState ref_fs;
  if (stf.GetFrenetStateFromState(state, &ref_fs) != kSuccess) {
    // printf("[GetSpeedLimit]Cannot get ref state frenet state.\n");
    return kWrongStatus;
  }

  // 使用固定 1 m/s^2 减速度估算提前执行限速所需的制动距离。
  const decimal_t acc_esti = 1.0;
  decimal_t limit = kInf;
  for (auto& speed_limit : speed_limit_list_) {
    // 逐信号判断参考 Lane 是否穿过其横向作用带，以及车辆相对纵向区间的位置。
    IntersectionType intersection_type;
    decimal_t dist_to_startpt;
    decimal_t dist_to_endpt;
    if (CheckIntersectionTypeWithSignal(ref_fs, lane, speed_limit,
                                        &intersection_type, &dist_to_startpt,
                                        &dist_to_endpt) != kSuccess) {
      continue;
    }
    if (intersection_type != kNotIntersect) {
      // 仅当当前速度高于该信号最大速度时才产生正的提前生效距离。
      decimal_t effect_speed_limit_dist =
          state.velocity > speed_limit.max_velocity()
              ? fabs(speed_limit.max_velocity() * speed_limit.max_velocity() -
                     state.velocity * state.velocity) /
                    (2.0 * acc_esti)
              : 0.0;
      // printf("[denny]cur vel %lf limit max vel %lf, effect dist %lf.\n",
      //        state.velocity, speed_limit.max_velocity(),
      //        effect_speed_limit_dist);
      // if (intersection_type == kSignalAhead)
      //   printf("[denny]Speed limit ahead.\n");
      // if (intersection_type == kSignalControlled)
      //   printf("[denny]under speed limit control.\n");
      // 已进入控制区时立即生效；尚在前方时，只有起点落入制动距离才生效。
      if ((intersection_type == kSignalAhead &&
           dist_to_startpt < effect_speed_limit_dist) ||
          intersection_type == kSignalControlled) {
        // 多个信号同时生效时取其最大允许速度中的最小值。
        limit = limit > speed_limit.max_velocity() ? speed_limit.max_velocity()
                                                   : limit;
      }
    }
  }
  // 没有任何生效信号时保持 kInf，调用方可继续采用其他速度约束。
  *speed_limit = limit;
  return kSuccess;
}

ErrorType TrafficSignalManager::CheckIntersectionTypeWithSignal(
    const common::FrenetState& fs, const Lane& lane,
    const common::TrafficSignal& signal, IntersectionType* intersection_type,
    decimal_t* dist_to_startpt, decimal_t* dist_to_endpt) const {
  // 将信号首尾世界坐标分别投影到参考 Lane，采用端点范围近似判断相交关系。
  common::StateTransformer stf(lane);
  Vec2f start_pt_fs, end_pt_fs;
  if (stf.GetFrenetPointFromPoint(signal.start_point(), &start_pt_fs) !=
      kSuccess) {
    return kWrongStatus;
  }
  if (stf.GetFrenetPointFromPoint(signal.end_point(), &end_pt_fs) != kSuccess) {
    return kWrongStatus;
  }

  Vec2f lateral_range = signal.lateral_range();
  IntersectionType int_type = kNotIntersect;
  decimal_t dist_to_start = 0.0;
  decimal_t dist_to_end = 0.0;
  // 只有投影后首点不晚于尾点，且首尾端点的横向作用区间都覆盖 Lane 中心 d=0，
  // 才把该信号视为与参考 Lane 相交。
  // std::cout << "[denny]start pt fs" << start_pt_fs.transpose() << std::endl;
  // std::cout << "[denny]end pt fs" << end_pt_fs.transpose() << std::endl;
  if (start_pt_fs[0] < end_pt_fs[0] + kEPS) {
    if (start_pt_fs[1] + lateral_range(1) > 0.0 &&
        start_pt_fs[1] + lateral_range(0) < 0.0 &&
        end_pt_fs[1] + lateral_range(1) > 0.0 &&
        end_pt_fs[1] + lateral_range(0) < 0.0) {
      if (fs.vec_s[0] < start_pt_fs[0]) {
        // 车辆位于信号起点之前，距离均为前向正向纵向距离。
        int_type = kSignalAhead;
        dist_to_start = start_pt_fs[0] - fs.vec_s[0];
        dist_to_end = end_pt_fs[0] - fs.vec_s[0];
      } else if (fs.vec_s[0] >= start_pt_fs[0] && fs.vec_s[0] < end_pt_fs[0]) {
        // 起点闭、终点开区间内视为已经受信号控制；起点距离为非正。
        int_type = kSignalControlled;
        dist_to_start = start_pt_fs[0] - fs.vec_s[0];
        dist_to_end = end_pt_fs[0] - fs.vec_s[0];
      } else {
        int_type = kNotIntersect;
      }
    }
  }
  // 不相交时类型为 kNotIntersect，两个距离保持初始化值 0。
  *intersection_type = int_type;
  *dist_to_startpt = dist_to_start;
  *dist_to_endpt = dist_to_end;
  return kSuccess;
}

ErrorType TrafficSignalManager::GetTrafficStoppingState(
    const State& state, const Lane& lane, State* stopping_state) const {
  // 交通灯/停车标志到停车状态的映射尚未实现，当前不会写入输出对象。
  return kSuccess;
}

}  // namespace semantic_map_manager
