/**
 * @file eudm_server_ros.h
 * @brief EUDM ROS2 地图队列、HMI 输入、独立规划线程和行为回调封装。
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#define _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__

#include <sensor_msgs/msg/joy.hpp>
#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "eudm_planner/visualizer.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include <rclcpp/rclcpp.hpp>
#include "semantic_map_manager/semantic_map_manager.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

/**
 * @brief 在 ROS2 节点上周期运行 EudmManager 并回传带行为的 SemanticMapManager。
 *
 * 地图生产者把快照写入有界 ReaderWriterQueue；独立线程按 work_rate 清空队列并只规划最新地图。
 * Joy 回调更新用户速度、拨杆、禁换和控制权，成功行为通过绑定回调交给下游轨迹规划器。
 */
class EudmPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  /// 跨线程语义地图队列容量。
  struct Config {
    int kInputBufferSize{100};
  };

  /// 使用默认 20 Hz 工作频率构造 server、visualizer 和输入队列。
  EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id);

  /// 使用指定工作频率构造 server、visualizer 和输入队列。
  EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id);

  /// 尝试把语义地图快照写入有界队列。
  void PushSemanticMap(const SemanticMapManager &smm);

  /// 绑定规划周期结束后接收更新地图的回调。
  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);

  /// 设置 Task 的用户期望速度。
  void set_user_desired_velocity(const decimal_t desired_vel);

  /// 返回 Task 中当前用户期望速度。
  decimal_t user_desired_velocity() const;

  /// 初始化 manager、Joy 订阅、节点参数和可视化 publisher。
  void Init(const std::string &bp_config_path);

  /// 启动 detached 规划线程并打开自动控制标志。
  void Start();

 private:
  /// 消费最新地图、运行 manager、更新行为、调用回调并发布可视化。
  void PlanCycleCallback();

  /// 解析当前 ego 的 Joy 按键并更新共享 Task。
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);

  /// 预留重规划入口；当前只有声明没有定义/调用。
  void Replan();

  /// 发布当前 planner 候选轨迹 MarkerArray。
  void PublishData();

  /// 按 work_rate 循环调用 PlanCycleCallback，直到 rclcpp 退出。
  void MainThread();

  /// 按相对时间查询动作序列中的当前动作；当前只有声明没有定义/调用。
  ErrorType GetCorrespondingActionInActionSequence(
      const decimal_t &t, const std::vector<DcpAction> &action_seq,
      DcpAction *a) const;

  /// server 固定配置。
  Config config_;

  /// 跨周期 manager 和借用其指针的可视化器。
  EudmManager bp_manager_;
  std::unique_ptr<EudmPlannerVisualizer> p_visualizer_;

  /// 规划线程持有的最新语义地图副本。
  SemanticMapManager smm_;

  /// Joy/外部接口更新、规划线程读取的用户任务。
  planning::eudm::Task task_;
  /// 传给 visualizer 的状态来源开关；当前 visualizer 未使用该值。
  bool use_sim_state_ = true;

  // ROS2 节点与 Joy 订阅。
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; 

  double work_rate_{20.0};
  int ego_id_;

  // 跨线程有界输入队列。
  std::unique_ptr<moodycamel::ReaderWriterQueue<SemanticMapManager>> p_input_smm_buff_;

  /// 下游行为更新回调及其绑定状态。
  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
