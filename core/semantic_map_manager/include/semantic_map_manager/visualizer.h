#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

/**
 * @brief 发布 SemanticMapManager 的自车、地图、Lane、行为、预测和交通信息可视化。
 *
 * 每个 node_id 使用独立 /vis/agent_<id>/... topic。Marker 主要位于 map 坐标系，并通过
 * 上一帧数量追加 DELETE Marker；另发布 map 到 ego_vehicle_vis_<id> 的 TF。
 */
class Visualizer {
 public:
  using ObstacleMapType = uint8_t;

  /// 保存 Node/node ID，创建 TF broadcaster 和九类 QoS 深度 1 的 publisher。
  Visualizer(rclcpp::Node::SharedPtr node, int node_id);

  /// publisher 和 Node 使用智能指针管理，析构函数为空。
  ~Visualizer() {}

  /// 使用 SMM 内部时间戳构造 ROS Time，发布全部图层并发送 ego TF。
  void VisualizeData(const SemanticMapManager &smm);

  /// 使用调用方时间戳发布全部图层，不发送 TF。
  void VisualizeDataWithStamp(const rclcpp::Time &stamp,
                              const SemanticMapManager &smm);

  /// 播放模式发布全部图层，并在 Lane 图层跳过指定已删除原始 Lane ID。
  void VisualizeDataWithStampForPlayback(
      const rclcpp::Time &stamp, const SemanticMapManager &smm,
      const std::vector<int> &deleted_lane_ids);

  /// 发布 map 到当前 ego 可视化 frame 的位姿变换。
  void SendTfWithStamp(const rclcpp::Time &stamp, const SemanticMapManager &smm);

 private:
  /// 发布自车 OBB、速度向量和转向 MarkerArray。
  void VisualizeEgoVehicle(const rclcpp::Time &stamp,
                           const common::Vehicle &vehicle);

  /// 发布周边原始 Lane 折线、首尾点和 Lane ID 文本，并处理删除列表。
  void VisualizeSurroundingLaneNet(const rclcpp::Time &stamp,
                                    const common::LaneNet &lane_net,
                                    const std::vector<int> &deleted_lane_ids);

  /// 发布自车 SemanticBehavior 中的参考 Lane、候选/周车轨迹等 Marker。
  void VisualizeBehavior(const rclcpp::Time &stamp,
                         const common::SemanticBehavior &behavior);

  /// 发布全部周车车辆 Marker；key/nearby 车辆使用更高不透明度。
  void VisualizeSurroundingVehicles(const rclcpp::Time &stamp,
                                    const common::VehicleSet &vehicle_set,
                                    const std::vector<int> &nearby_ids);

  /// 发布拟合后的本地 Lane 面片，并过滤包含已删除原始 Lane 的本地 Lane。
  void VisualizeLocalLanes(
      const rclcpp::Time &stamp,
      const std::unordered_map<int, common::Lane> &local_lanes,
      const SemanticMapManager &smm,
      const std::vector<int> &deleted_lane_ids);

  /// 将内部 GridMap 转换为 OccupancyGrid 后发布。
  void VisualizeObstacleMap(
      const rclcpp::Time &stamp,
      const common::GridMapND<ObstacleMapType, 2> &obstacle_map);

  /// 用概率长度箭头显示每辆语义周车的 LK/LCL/LCR 意图分布。
  void VisualizeIntentionPrediction(
      const rclcpp::Time &stamp, const common::SemanticVehicleSet &s_vehicle_set);

  /// 用圆柱采样点和折线发布每辆周车的开环预测轨迹。
  void VisualizeOpenloopTrajPrediction(
      const rclcpp::Time &stamp,
      const std::unordered_map<int, vec_E<common::State>> &openloop_pred_trajs);

  /// 为每个限速区间发布起终点标牌和文本；零限速显示为红灯/禁行。
  void VisualizeSpeedLimit(const rclcpp::Time &stamp,
                           const vec_E<common::SpeedLimit> &speed_limits);

  // 各变长 MarkerArray 上一帧的实际 ADD 数量，用于删除残留 ID。
  int last_traj_list_marker_cnt_ = 0;
  int last_intention_marker_cnt_ = 0;
  int last_surrounding_vehicle_marker_cnt_ = 0;
  int last_speed_limit_marker_cnt_ = 0;
  int last_surrounding_lanes_cnt_ = 0;
  int last_behavior_marker_cnt_ = 0;

  // 当前 ego TF 子 frame 名称。
  std::string ego_tf_name_;

  // 外部共享 Node 与当前 agent/node ID。
  rclcpp::Node::SharedPtr node_;
  int node_id_;

  // 九个独立图层 publisher。
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego_vehicle_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surrounding_lane_net_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_lanes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr behavior_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_traj_openloop_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_intention_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surrounding_vehicle_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr speed_limit_vis_pub_;

  // 构造时创建的 TF broadcaster；当前 SendTfWithStamp 未使用该成员。
  tf2_ros::TransformBroadcaster ego_to_map_tf_;

  // 预留 Marker 生命周期；当前各 Marker 构造路径未读取该值。
  decimal_t marker_lifetime_{0.05};
};  // Visualizer

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_
