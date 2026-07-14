#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_

#include <assert.h>

#include <functional>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "rclcpp/rclcpp.hpp"
#include "semantic_map_manager/data_renderer.h"
#include "vehicle_msgs/decoder.h"

#include "vehicle_msgs/msg/arena_info.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"

namespace semantic_map_manager {

/**
 * @brief 把物理仿真 ArenaInfo ROS2 消息解码为内部对象并驱动 DataRenderer。
 *
 * Adapter 同时支持完整 arena_info，以及 static Lane/障碍物加 dynamic 车辆的拆分消息。
 * 每次成功进入渲染路径后，可在 ROS executor 回调线程中通知上层最新 SemanticMapManager。
 */
class RosAdapter {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;

  /// 保存共享 Node 和非拥有 SMM 指针，创建 DataRenderer，并立即订阅三类 ArenaInfo。
  RosAdapter(std::shared_ptr<rclcpp::Node> node, SemanticMapManager* ptr_smm)
      : node_(node), p_smm_(ptr_smm), p_data_renderer_(new DataRenderer(ptr_smm)) {
    Init();
  }

  /// 删除本类拥有的 DataRenderer；SMM 和 Node 生命周期仍由外部管理。
  ~RosAdapter() {
    delete p_data_renderer_;
  }

  /// 绑定语义地图更新回调；回调在触发对应 ArenaInfo 的 executor 线程中执行。
  void BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn);

  /// 创建完整、静态和动态 ArenaInfo 三个相对 topic 订阅，QoS 深度均为 2。
  void Init();

 private:
  /// 解码完整 LaneNet/车辆/障碍物消息，立即渲染并通知回调。
  void ArenaInfoCallback(const vehicle_msgs::msg::ArenaInfo::SharedPtr msg);

  /// 解码并缓存静态 LaneNet/障碍物，标记拆分输入已经就绪。
  void ArenaInfoStaticCallback(const vehicle_msgs::msg::ArenaInfoStatic::SharedPtr msg);

  /// 解码动态车辆；收到过静态消息后，与最近静态缓存组合渲染并通知回调。
  void ArenaInfoDynamicCallback(const vehicle_msgs::msg::ArenaInfoDynamic::SharedPtr msg);

  // 外部共享 ROS2 Node。
  std::shared_ptr<rclcpp::Node> node_;

  // 与物理仿真通信的完整、静态和动态 ArenaInfo 订阅。
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfo>::SharedPtr arena_info_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr arena_info_static_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr arena_info_dynamic_sub_;

  // 最近一次解码得到的内部缓存；ego_vehicle_ 当前未在 adapter 中使用。
  common::Vehicle ego_vehicle_;
  common::VehicleSet vehicle_set_;
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;

  // 本类拥有的 DataRenderer 与非拥有 SemanticMapManager 回写目标。
  DataRenderer* p_data_renderer_;
  SemanticMapManager* p_smm_;

  // 拆分输入路径是否至少收到过一次静态 Lane/障碍物消息。
  bool get_arena_info_static_ = false;

  // 可选地图更新回调及启用标记。
  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager&)> private_callback_fn_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
