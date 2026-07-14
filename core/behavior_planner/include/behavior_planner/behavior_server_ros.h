#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "behavior_planner/behavior_planner.h"
#include "behavior_planner/map_adapter.h"
#include "behavior_planner/visualizer.h"
#include "semantic_map_manager/semantic_map_manager.h"

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"

namespace planning {

/**
 * @brief BehaviorPlanner 的 ROS2 运行封装与异步语义地图消费服务。
 *
 * 上层通过 PushSemanticMap 写入有界队列；独立规划线程按 work_rate_ 周期清空队列并只
 * 使用最新地图，运行 MPDM 后通过回调返回更新的 SemanticMapManager，同时发布调试轨迹。
 */
class BehaviorPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;

  /// Server 内部固定配置；当前只包含语义地图输入队列容量。
  struct Config {
    int kInputBufferSize{100};
  };

  /// 以默认 20 Hz 工作频率构造 server，并创建可视化器和输入队列。
  BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id);

  /// 以调用方指定频率构造 server；当前不校验 node、频率或 ego ID。
  BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id);

  /// 尝试把语义地图值拷贝压入有界队列；队列不存在或写入失败时静默丢弃。
  void PushSemanticMap(const SemanticMapManager &smm);

  /// 绑定规划结果回调；回调在 server 的独立规划线程中执行。
  void BindBehaviorUpdateCallback(std::function<int(const SemanticMapManager &)> fn);

  /// 转发设置 BehaviorPlanner 自动驾驶等级。
  void set_autonomous_level(int level);

  /// 转发设置用户期望速度。
  void set_user_desired_velocity(const decimal_t desired_vel);

  /// 转发设置前向仿真的驾驶激进程度。
  void set_aggressive_level(int level);

  /// 返回 BehaviorPlanner 当前用户期望速度。
  decimal_t user_desired_velocity() const;

  /// 返回 BehaviorPlanner 当前曲率约束参考速度。
  decimal_t reference_desired_velocity() const;

  /// 允许 JoyCallback 接受本 ego 的 HMI 指令。
  void enable_hmi_interface();

  /// 初始化规划器、Joy 订阅、状态源参数和可视化 publisher。
  void Init();

  /// 注入地图适配器并启动 detached 规划线程。
  void Start();

 private:
  /// 单周期清空输入队列，使用最新语义地图运行规划、回调并发布可视化。
  void PlanCycleCallback();

  /// 解析本 ego 的 Joy 按键，触发左右换道或以 1 m/s 步长调整期望速度。
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);

  /// 预留的显式重规划入口；当前为空实现。
  void Replan();

  /// 使用当前 ROS 时钟发布 BehaviorPlanner 调试轨迹。
  void PublishData();

  /// 按 work_rate_ 循环调用 PlanCycleCallback，直到 rclcpp 退出。
  void MainThread();

  Config config_;

  // 行为规划核心、共享语义地图适配器，以及读取规划器调试缓存的可视化器。
  BehaviorPlanner bp_;
  BehaviorPlannerMapAdapter map_adapter_;
  std::unique_ptr<BehaviorPlannerVisualizer> p_visualizer_;

  // 预留的性能统计对象与全局起始时间戳；当前 server 未使用。
  TicToc time_profile_tool_;
  decimal_t global_init_stamp_{0.0};

  // 外部共享 ROS2 节点和可选 Joy 订阅。
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // 本 server 所属 ego ID 与独立线程的目标工作频率。
  int ego_id_;
  double work_rate_;

  // 上层生产、规划线程消费的有界语义地图队列。
  std::unique_ptr<moodycamel::ReaderWriterQueue<SemanticMapManager>> p_input_smm_buff_;

  // 可选结果回调及其启用标记。
  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;

  // Joy 消息还需显式打开此开关才会转为 HMI 命令。
  bool is_hmi_enabled_ = false;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__
