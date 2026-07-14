#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

/**
 * @brief 将物理仿真车辆、LaneNet 和障碍物发布为 map frame MarkerArray。
 *
 * Visualizer 借用外部 PhySimulation；三个 publisher 使用固定绝对 topic。
 */
class Visualizer {
 public:
  /// 默认构造不创建 publisher 或绑定仿真器。
  Visualizer() {}
  /// 使用 node 创建车辆、LaneNet 和障碍物 MarkerArray publisher。
  Visualizer(rclcpp::Node::SharedPtr node);
  /// shared publisher 自动释放，析构为空。
  ~Visualizer() {}

  /// 绑定外部 PhySimulation 借用指针。
  void set_phy_sim(PhySimulation *p_phy_sim) { p_phy_sim_ = p_phy_sim; }

  /// 使用 node 当前时钟发布三类可视化。
  void VisualizeData();
  /// 使用指定时间戳发布三类可视化。
  void VisualizeDataWithStamp(const rclcpp::Time &stamp);
  /// 预留 TF 发布接口；当前只有声明、没有实现。
  void SendTfWithStamp(const rclcpp::Time &stamp);

 private:
  /// 为每辆车发布 OBB、速度和转向 Marker。
  void VisualizeVehicleSet(const rclcpp::Time &stamp,
                           const common::VehicleSet &vehicle_set);
  /// 发布 Lane 中心线、首尾点和 ID 文本。
  void VisualizeLaneNet(const rclcpp::Time &stamp,
                         const common::LaneNet &lane_net);
  /// 发布全部 PolygonObstacle。
  void VisualizeObstacleSet(const rclcpp::Time &stamp,
                             const common::ObstacleSet &Obstacle_set);

  // ROS2 node 和三类 Marker publisher。
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_set_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_net_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_set_pub_;

  /// 外部仿真器借用指针。
  PhySimulation *p_phy_sim_;
};  // Visualizer

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_
