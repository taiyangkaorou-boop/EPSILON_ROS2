#ifndef _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
#define _UTIL_SSC_PLANNER_INC_VISUALIZER_H_

#include <assert.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/primitive/frenet_primitive.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "ssc_planner/ssc_planner.h"

namespace planning {

/**
 * @brief 将 SSC 的 `(s,d,t)` 占用、rollout、corridor 和 QP 轨迹发布为 MarkerArray。
 *
 * 可视化统一使用 `ssc_map` frame，并把绝对轨迹时间减去本轮 time_origin 后映射到 z 轴。
 */
class SscVisualizer {
 public:
  /// 按自车 ID 创建六组独立可视化 topic publisher。
  SscVisualizer(rclcpp::Node::SharedPtr node, int node_id);
  /// publisher 由 shared_ptr 自动管理，析构为空。
  ~SscVisualizer() {}

  /// 从规划器快照依次发布地图、自车、候选轨迹、周车、走廊和 QP spline。
  void VisualizeDataWithStamp(const rclcpp::Time &stamp, const SscPlanner &planner);

 private:
  /// 发布三维占用栅格和道路范围辅助 AABB。
  void VisualizeSscMap(const rclcpp::Time &stamp, const SscMap *p_ssc_map);
  /// 发布起始自车 Frenet 车身轮廓和参考点。
  void VisualizeEgoVehicleInSscSpace(const rclcpp::Time &stamp, const common::FsVehicle &fs_ego_vehicle);
  /// 发布全部候选自车 Frenet rollout 的车身轮廓和状态点。
  void VisualizeForwardTrajectoriesInSscSpace(
      const rclcpp::Time &stamp, const vec_E<vec_E<common::FsVehicle>> &trajs,
      const SscMap *p_ssc_map);
  /// 采样并发布全部五阶二维 Bezier 候选轨迹。
  void VisualizeQpTrajs(const rclcpp::Time &stamp, const vec_E<common::BezierSpline<5, 2>> &trajs);
  /// 发布第一组候选行为中的周车 Frenet rollout。
  void VisualizeSurroundingVehicleTrajInSscSpace(
      const rclcpp::Time &stamp,
      const vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> &trajs_set,
      const SscMap *p_ssc_map);
  /// 发布所有 DrivingCorridor 的 seed 点与轴对齐 cube。
  void VisualizeCorridorsInSscSpace(
      const rclcpp::Time &stamp, const vec_E<common::DrivingCorridor> corridor_vec,
      const SscMap *p_ssc_map);

  // 预留的历史 Marker 计数；当前两个成员没有被使用。
  int last_traj_list_marker_cnt_ = 0;
  int last_surrounding_vehicle_marker_cnt_ = 0;

  rclcpp::Node::SharedPtr node_;
  int node_id_;

  /// 当前规划轨迹的绝对时间原点，用于把 t 转为 Marker z 高度。
  decimal_t start_time_;

  // 六类 SSC 诊断 topic publisher。
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ssc_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego_vehicle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_trajs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sur_vehicle_trajs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr qp_pub_;

  // 各动态 MarkerArray 上一帧数量，用于辅助删除残留 Marker。
  int last_corridor_mk_cnt = 0;
  int last_qp_traj_mk_cnt = 0;
  int last_sur_vehicle_traj_mk_cnt = 0;
  int last_forward_traj_mk_cnt = 0;
};  // SscVisualizer
}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
