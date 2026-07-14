/**
 * @file route_planner.h
 * @brief 基于 LaneNet 随机后继扩展或指定目标的轻量导航 Lane 生成器。
 */

#ifndef _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_
#define _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_

#include <memory>
#include <random>
#include <set>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"

namespace planning {

/**
 * @brief 为 BehaviorPlanner 生成 Lane ID 导航路径和连续导航参考 Lane。
 *
 * 默认从自车最近 Lane 开始随机选择 child，累计约 200 m 后拼接中心线；状态机维护 ready、
 * in-progress、finished。指定目标模式当前仅保留接口。
 */
class RoutePlanner : public Planner {
 public:
  /// 导航路径生成方式。
  enum NaviMode { kRandomExpansion, kAssignedTarget };
  /// 单次导航任务生命周期。
  enum NaviStatus { kReadyToGo, kInProgress, kFinished };

  /// 返回规划器名称。
  std::string Name() override;

  /// 重置导航状态；config 当前未解析。
  ErrorType Init(const std::string config) override;

  /// 按当前模式推进状态机，并要求输出 navi_lane 有效。
  ErrorType RunOnce() override;

  /// 设置导航模式。
  void set_navi_mode(const NaviMode& mode) { navi_mode_ = mode; };
  /// 设置本周期自车状态。
  void set_ego_state(const common::State& state) { ego_state_ = state; }
  /// 设置本周期自车最近 Lane ID。
  void set_nearest_lane_id(const int& id) { nearest_lane_id_ = id; }
  /// 按值缓存完整 LaneNet，并标记地图已获取。
  void set_lane_net(const common::LaneNet& lane_net) {
    lane_net_ = lane_net;
    if_get_lane_net_ = true;
  }

  /// 按值返回随机/目标导航 Lane ID 路径。
  std::vector<int> navi_path() const { return navi_path_; }
  /// 按值返回拼接拟合后的连续导航 Lane。
  common::Lane navi_lane() const { return navi_lane_; }

  /// 返回是否已经设置 LaneNet。
  bool if_get_lane_net() const { return if_get_lane_net_; }

  /// 返回自车在导航 Lane 上的当前弧长缓存。
  decimal_t navi_cur_arc_len() const { return navi_cur_arc_len_; }

 private:
  /// 查询 Lane 的全部 child ID，覆盖输出容器。
  ErrorType GetChildLaneIds(const int lane_id, std::vector<int>* child_ids);

  /// 随机扩展 child Lane，拼接中心线并初始化导航长度/弧长。
  ErrorType GetNaviPathByRandomExpansion();

  /// 判断最近 Lane 是否等于导航路径尾 Lane；当前没有调用点。
  bool CheckIfArriveTargetLane();
  /// 更新导航任务进度；当前实现无条件置 finished。
  ErrorType CheckNaviProgress();

  /// 推进随机扩展状态机。
  ErrorType NaviLoopRandomExpansion();
  /// 推进指定目标状态机；当前为空实现。
  ErrorType NaviLoopAssignedTarget();

  /// 完整 LaneNet 和本周期自车状态。
  common::LaneNet lane_net_;
  common::State ego_state_;

  /// 自车最近 Lane ID；当前无类内默认值。
  int nearest_lane_id_;

  /// 导航状态和模式。
  NaviStatus navi_status_ = kReadyToGo;
  NaviMode navi_mode_ = kRandomExpansion;

  /// 完成后是否自动重启，以及 LaneNet 是否已注入。
  bool if_restart_ = true;
  bool if_get_lane_net_ = false;

  /// 随机路径目标长度和自车在拟合 Lane 上的进度量。
  decimal_t navi_path_max_length_ = 200;
  decimal_t navi_start_arc_length_{0.0};
  decimal_t navi_path_length_{0.0};
  decimal_t navi_cur_arc_len_{0.0};

  /// 导航 Lane ID 序列和连续 Lane 几何。
  std::vector<int> navi_path_;
  common::Lane navi_lane_;

  /// 直接用于 child 选择的非确定性 random_device。
  std::random_device rd_gen_;
};

}  // namespace planning

#endif  //_CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_
