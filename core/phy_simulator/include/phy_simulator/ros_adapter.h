/**
 * @file ros_adapter.h
 * @brief 将 PhySimulation 快照编码并发布为 ROS2 ArenaInfo 消息。
 * @version 0.1
 * @date 2019-03-18
 *
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_msgs/encoder.h"

#include "common/basics/basics.h"
#include "common/basics/semantics.h"

#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

/**
 * @brief 物理仿真器到 vehicle_msgs 静态/动态真值消息的发布适配器。
 *
 * 类借用外部 PhySimulation，不负责其生命周期；所有消息使用调用者时间戳和固定 map frame。
 */
class RosAdapter {
 public:
  /// 默认构造不创建 publishers，也不绑定仿真器。
  RosAdapter();

  /// 使用 ROS2 node 创建完整、静态和动态 ArenaInfo publishers。
  RosAdapter(std::shared_ptr<rclcpp::Node> node);

  /// 绑定外部 PhySimulation 借用指针。
  void set_phy_sim(PhySimulation *p_phy_sim) { p_phy_sim_ = p_phy_sim; }

  /// 编码并发布 LaneNet、VehicleSet 和 ObstacleSet 的完整 ArenaInfo。
  void PublishDataWithStamp(const rclcpp::Time &stamp);

  /// 只编码并发布 VehicleSet 动态真值。
  void PublishDynamicDataWithStamp(const rclcpp::Time &stamp);

  /// 编码并发布 LaneNet 与 ObstacleSet 静态真值。
  void PublishStaticDataWithStamp(const rclcpp::Time &stamp);

 private:
  // ROS2 node 和三个深度 10 publisher。
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfo>::SharedPtr arena_info_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr arena_info_static_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr arena_info_dynamic_pub_;

  /// 外部仿真器借用指针。
  PhySimulation *p_phy_sim_;
};  // RosAdapter
}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_
