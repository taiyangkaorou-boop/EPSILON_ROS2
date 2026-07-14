// BehaviorPlanner 候选轨迹的 ROS2 MarkerArray 可视化封装。
#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <functional>
#include <iostream>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vehicle_msgs/decoder.h"

#include "behavior_planner/behavior_planner.h"
#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

/**
 * @brief 把 BehaviorPlanner 的有效自车候选 rollout 发布为 RViz 圆柱采样点和折线。
 *
 * 本类共享 ROS2 Node，并以非拥有裸指针读取 BehaviorPlanner 调试缓存。每个 ego 使用独立
 * topic；Marker 统一放在 map 坐标系，并通过上一帧数量删除已经消失的 Marker ID。
 */
class BehaviorPlannerVisualizer {
 public:
  /// 保存共享节点、ego ID 和非拥有规划器指针；publisher 由 Init 单独创建。
  BehaviorPlannerVisualizer(std::shared_ptr<rclcpp::Node> node, BehaviorPlanner* ptr_bp, int ego_id)
      : node_(node), ego_id_(ego_id) {
    p_bp_ = ptr_bp;
  }

  /// 创建 /vis/agent_<ego_id>/forward_trajs MarkerArray publisher，QoS 深度为 1。
  void Init() {
    // 每个 ego 使用独立绝对 topic，避免多车候选轨迹发布到同一通道。
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");
    forward_traj_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(forward_traj_topic, 1);
  }

  /// 读取当前全部有效候选轨迹并使用给定 ROS 时间戳发布 MarkerArray。
  void PublishDataWithStamp(const rclcpp::Time& stamp) {
    if (p_bp_ == nullptr) return;
    // getter 返回完整值拷贝；当前帧所有候选统一使用 gold 颜色。
    auto forward_trajs = p_bp_->forward_trajs();
    visualization_msgs::msg::MarkerArray traj_list_marker;
    common::ColorARGB traj_color = common::cmap.at("gold");
    for (const auto& traj : forward_trajs) {
      // 一条候选轨迹先生成离散圆柱 Marker，同时收集折线所需的二维位置点。
      std::vector<common::Point> points;
      for (const auto& v : traj) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        pt.z = 0.3;
        points.push_back(pt);
        visualization_msgs::msg::Marker point_marker;
        common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
            common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
            &point_marker);
        traj_list_marker.markers.push_back(point_marker);
      }
      // 每个候选无论状态数量多少都追加一条连接全部采样点的 LineStrip。
      visualization_msgs::msg::Marker line_marker;
      common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
          points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
      traj_list_marker.markers.push_back(line_marker);
    }
    // 记录填充 header/id 前的真实 Marker 数量，不把随后追加的 DELETE Marker 计入下一帧。
    int num_markers = static_cast<int>(traj_list_marker.markers.size());
    // 统一写入 map frame、时间戳和连续 ID，并为上一帧多出的 ID 追加删除消息。
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;
    forward_traj_vis_pub_->publish(traj_list_marker);
  }

 private:
  // 外部共享 ROS2 节点和当前可视化器所属 ego ID。
  std::shared_ptr<rclcpp::Node> node_;
  int ego_id_;

  // 上一帧实际 ADD Marker 数量，以及本 ego 的候选轨迹 publisher。
  int last_forward_trajs_marker_cnt_ = 0;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_traj_vis_pub_;

  // 非拥有 BehaviorPlanner 指针，调用方必须保证其生命周期覆盖本可视化器。
  BehaviorPlanner* p_bp_{nullptr};
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_
