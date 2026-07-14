/**
 * @file visualizer.h
 * @brief EUDM 全部候选轨迹及原始/重选 winner 的 ROS2 MarkerArray 可视化。
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <functional>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

/**
 * @brief 把 EudmPlanner 候选 rollout 发布为 RViz 圆柱采样点和折线。
 *
 * 可视化器共享 ROS2 Node，并以非拥有指针读取 EudmManager/planner 调试缓存；processed winner
 * 使用 gold，若不同则 original winner 使用 spring green，其余候选使用半透明灰色。
 */
class EudmPlannerVisualizer {
 public:
  /// 保存共享节点、ego ID 和非拥有 manager 指针；publisher 由 Init 创建。
  EudmPlannerVisualizer(std::shared_ptr<rclcpp::Node> node, EudmManager* p_bp_manager, int ego_id)
      : node_(node), ego_id_(ego_id) {
    assert(p_bp_manager != nullptr);
    p_bp_manager_ = p_bp_manager;
  }

  /// 创建 /vis/agent_<ego_id>/forward_trajs publisher，QoS 深度为 1。
  void Init() {
    // 每个 ego 使用独立绝对 topic。
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");

    forward_traj_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(forward_traj_topic, 1);
  }

  /// 使用给定 ROS 时间戳发布当前全部候选轨迹。
  void PublishDataWithStamp(const rclcpp::Time& stamp) {
    VisualizeForwardTrajectories(stamp);
  }

  /// 生成每个候选的离散圆柱和折线 Marker，并处理上一帧残留 ID。
  void VisualizeForwardTrajectories(const rclcpp::Time& stamp) {
    // planner getter 会按值复制全部候选轨迹；winner ID 来自 manager 最后快照。
    auto forward_trajs = p_bp_manager_->planner().forward_trajs();
    int processed_winner_id = p_bp_manager_->processed_winner_id();
    int original_winner_id = p_bp_manager_->original_winner_id();
    visualization_msgs::msg::MarkerArray traj_list_marker;
    common::ColorARGB traj_color(0.5, 0.5, 0.5, 0.5);
    double traj_z = 0.3;
    for (int i = 0; i < static_cast<int>(forward_trajs.size()); ++i) {
      // processed winner 优先着金色；不同的 original winner 着绿色。
      if (i == processed_winner_id) {
        traj_color = common::cmap.at("gold");
        traj_z = 0.4;
      } else if (i == original_winner_id) {
        traj_color = common::cmap.at("spring green");
        traj_z = 0.4;
      } else {
        traj_color = common::ColorARGB(0.5, 0.5, 0.5, 0.5);
        traj_z = 0.3;
      }
      // 一条候选生成逐状态圆柱，并收集 LineStrip 的位置点。
      std::vector<common::Point> points;
      for (const auto& v : forward_trajs[i]) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        pt.z = traj_z;
        points.push_back(pt);
        visualization_msgs::msg::Marker point_marker;
        common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
            common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
            &point_marker);
        traj_list_marker.markers.push_back(point_marker);
      }
      visualization_msgs::msg::Marker line_marker;
      common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
          points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
      traj_list_marker.markers.push_back(line_marker);
    }
    // FillHeaderId 会统一设置 map frame/时间/ID，并删除上一帧多出的 Marker。
    int num_markers = static_cast<int>(traj_list_marker.markers.size());
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;
    forward_traj_vis_pub_->publish(traj_list_marker);
  }

  /// 缓存状态来源开关；当前绘制逻辑没有读取该成员。
  void set_use_sim_state(bool use_sim_state) { use_sim_state_ = use_sim_state; }

 private:
  /// 共享 ROS2 节点、可视化所属 ego 和未使用的状态来源开关。
  std::shared_ptr<rclcpp::Node> node_;
  int ego_id_;
  bool use_sim_state_ = true;

  /// 上一帧实际 ADD Marker 数量。
  int last_forward_trajs_marker_cnt_ = 0;

  /// 候选轨迹 publisher。
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_traj_vis_pub_;
  /// 非拥有 manager 指针，server 必须保证其生命周期覆盖可视化器。
  EudmManager* p_bp_manager_{nullptr};

};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
