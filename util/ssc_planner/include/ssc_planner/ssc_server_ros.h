#ifndef _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_

#include <chrono>
#include <memory>
#include <numeric>
#include <thread>

#include "common/basics/colormap.h"
#include "common/basics/tic_toc.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/trajectory/frenet_traj.h"
#include "common/visualization/common_visualization_util.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include "rclcpp/rclcpp.hpp"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/map_adapter.h"
#include "ssc_planner/ssc_planner.h"
#include "ssc_planner/ssc_visualizer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

/**
 * @brief SSC 规划器的 ROS2 调度、轨迹拼接、控制反馈与可视化服务端。
 *
 * 外部线程把 SemanticMapManager 快照写入无锁队列；后台线程按 work_rate 取最新快照，
 * 驱动首轮规划或沿执行轨迹量化重规划，并发布控制信号、执行轨迹和诊断 Marker。
 */
class SscPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using FrenetTrajectory = common::FrenetTrajectory;
  /// 服务端输入队列配置。
  struct Config {
    int kInputBufferSize{100};
  };

  /// 使用默认 20 Hz 工作频率构造服务端及两个可视化器。
  SscPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id);

  /// 使用指定工作频率构造服务端及两个可视化器。
  SscPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id);

  /// 尝试把语义地图值拷贝写入单生产者/单消费者队列。
  void PushSemanticMap(const SemanticMapManager &smm);

  /// 初始化规划器配置、参数和 ROS2 publishers。
  void Init(const std::string &config_path);

  /// 绑定地图适配器并启动 detached 后台规划线程；重复调用直接返回。
  void Start();


 private:
  /// 消费最新地图，并推进初始化、重规划或轨迹切换状态机。
  void PlanCycleCallback();

  /// 沿当前执行轨迹选择下一周期起点并生成 next_traj_。
  void Replan();

  /// 发布语义/SSC 可视化、执行轨迹控制和轨迹 Marker。
  void PublishData();

  /// 按 work_rate 循环调用 PlanCycleCallback。
  void MainThread();

  /// 极低速且姿态跳变过大时，用上一历史状态姿态覆盖当前姿态。
  ErrorType FilterSingularityState(const vec_E<common::State> &hist, 
                                   common::State *filter_state);

  Config config_;

  // 服务状态、地图更新状态和是否从执行轨迹发布仿真控制。
  bool is_replan_on_ = false;
  bool is_map_updated_ = false;
  bool use_sim_state_ = true;
  std::unique_ptr<FrenetTrajectory> executing_traj_;
  std::unique_ptr<FrenetTrajectory> next_traj_;

  // SSC 核心规划器及其语义地图适配器。
  SscPlanner planner_;
  SscPlannerAdapter map_adapter_;

  TicToc time_profile_tool_;
  decimal_t global_init_stamp_{0.0};

  // ROS2 节点、调度频率、自车 ID 和发布器。
  std::shared_ptr<rclcpp::Node> node_;
  decimal_t work_rate_ = 20.0;
  int ego_id_;

  bool require_intervention_signal_ = false;
  rclcpp::Publisher<vehicle_msgs::msg::ControlSignal>::SharedPtr ctrl_signal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr executing_traj_vis_pub_;

  // SemanticMapManager 输入队列和最近一次成功出队的完整快照。
  std::unique_ptr<moodycamel::ReaderWriterQueue<SemanticMapManager>> p_input_smm_buff_ {nullptr};

  SemanticMapManager last_smm_;
  // 语义地图/SSC 可视化器及上一帧执行轨迹 Marker 数量。
  std::unique_ptr<semantic_map_manager::Visualizer> p_smm_vis_ {nullptr};
  std::unique_ptr<SscVisualizer> p_ssc_vis_;
  int last_trajmk_cnt_{0};

  // 重规划目标状态和控制反馈状态的有限历史窗口。
  vec_E<common::State> desired_state_hist_;
  vec_E<common::State> ctrl_state_hist_;
};

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_
