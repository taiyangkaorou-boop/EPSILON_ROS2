#include "semantic_map_manager/ros_adapter.h"

namespace semantic_map_manager {

void RosAdapter::Init() {
  // 同时订阅完整消息和静态/动态拆分消息；当前没有输入模式互斥选择。
  arena_info_sub_ = node_->create_subscription<vehicle_msgs::msg::ArenaInfo>(
      "arena_info", 2, std::bind(&RosAdapter::ArenaInfoCallback, this, std::placeholders::_1));
  arena_info_static_sub_ = node_->create_subscription<vehicle_msgs::msg::ArenaInfoStatic>(
      "arena_info_static", 2, std::bind(&RosAdapter::ArenaInfoStaticCallback, this, std::placeholders::_1));
  arena_info_dynamic_sub_ = node_->create_subscription<vehicle_msgs::msg::ArenaInfoDynamic>(
      "arena_info_dynamic", 2, std::bind(&RosAdapter::ArenaInfoDynamicCallback, this, std::placeholders::_1));
}

void RosAdapter::ArenaInfoCallback(const vehicle_msgs::msg::ArenaInfo::SharedPtr msg) {
  // 完整消息一次性覆盖 LaneNet、VehicleSet 和 ObstacleSet，并输出消息时间戳。
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
      *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
  // Decoder 和 Render 返回码均被忽略，随后仍可能通知上层当前 SMM。
  p_data_renderer_->Render(time_stamp.seconds(), lane_net_, vehicle_set_, obstacle_set_);
  if (has_callback_binded_) {
    // 回调在当前 ROS executor 回调线程同步执行，返回值被忽略。
    private_callback_fn_(*p_smm_);
  }
}

void RosAdapter::ArenaInfoStaticCallback(const vehicle_msgs::msg::ArenaInfoStatic::SharedPtr msg) {
  // 静态消息只刷新 LaneNet 和障碍物缓存；其时间戳不会参与后续静/动态同步判断。
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  // 不论解码是否成功都把静态输入标记为已就绪。
  get_arena_info_static_ = true;
}

void RosAdapter::ArenaInfoDynamicCallback(const vehicle_msgs::msg::ArenaInfoDynamic::SharedPtr msg) {
  // 动态消息刷新车辆集合和本次渲染时间戳。
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(*msg, &time_stamp, &vehicle_set_);

  // 至少收到一次静态消息后，使用最新车辆与最近静态 Lane/障碍物组合渲染。
  if (get_arena_info_static_) {
    p_data_renderer_->Render(time_stamp.seconds(), lane_net_, vehicle_set_, obstacle_set_);
    if (has_callback_binded_) {
      // 与完整消息路径相同，回调同步执行且返回值被忽略。
      private_callback_fn_(*p_smm_);
    }
  }
}

void RosAdapter::BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn) {
  // 保存回调包装并置启用标记；当前不检查空函数或并发绑定。
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager
