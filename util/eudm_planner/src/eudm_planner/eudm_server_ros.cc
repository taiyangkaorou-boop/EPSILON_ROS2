/**
 * @file eudm_server_ros.cc
 * @brief EUDM ROS2 server 构造、Joy HMI、地图消费和独立规划循环实现。
 */

#include "eudm_planner/eudm_server_ros.h"

namespace planning {

// 使用默认 20 Hz 创建可视化器和容量为 100 的语义地图队列。
EudmPlannerServer::EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id)
    : node_(node), work_rate_(20.0), ego_id_(ego_id) {
  p_visualizer_ = std::make_unique<EudmPlannerVisualizer>(node, &bp_manager_, ego_id);
  p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

// 使用调用方工作频率创建可视化器和语义地图队列。
EudmPlannerServer::EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id)
    : node_(node), work_rate_(work_rate), ego_id_(ego_id) {
  p_visualizer_ = std::make_unique<EudmPlannerVisualizer>(node, &bp_manager_, ego_id);
  p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

// 尝试写入输入队列；队列满时 try_enqueue 失败但当前不记录丢帧。
void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

// 使用节点当前 ROS 时间发布 planner 候选轨迹。
void EudmPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(node_->get_clock()->now());
}

// 初始化 manager、Joy 订阅、use_sim_state 参数和可视化 publisher。
void EudmPlannerServer::Init(const std::string &bp_config_path) {
  bp_manager_.Init(bp_config_path, work_rate_);
  auto joy_callback = std::bind(&EudmPlannerServer::JoyCallback, this, std::placeholders::_1);
  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, joy_callback);
  // 参数未在本类声明，依赖节点外部已声明或允许 undeclared parameter。
  // node_->declare_parameter("use_sim_state", use_sim_state_);
  node_->get_parameter("use_sim_state", use_sim_state_);
  p_visualizer_->Init();
  p_visualizer_->set_use_sim_state(use_sim_state_);
}

// 处理本 ego 的 Joy 按键：拨杆、速度、左右禁换和自动控制开关。
void EudmPlannerServer::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  int msg_id;
  // 空 frame_id 视为 ego 0，否则把字符串解析为目标 ego ID。
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) return;
  // buttons: 2 左换、1 右换、3 加速、0 减速、4/5 左右禁换、6 控制权。
  // 当前直接访问 0..6，依赖消息至少提供 7 个按键。
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;

  // 左/右拨杆再次按下会在目标方向和 0 之间切换。
  if (msg->buttons[2] == 1) {
    if (task_.user_perferred_behavior != -1) {
      task_.user_perferred_behavior = -1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[1] == 1) {
    if (task_.user_perferred_behavior != 1) {
      task_.user_perferred_behavior = 1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[3] == 1) {
    task_.user_desired_vel = task_.user_desired_vel + 1.0;
  } else if (msg->buttons[0] == 1) {
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0);
  } else if (msg->buttons[4] == 1) {
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left;
  } else if (msg->buttons[5] == 1) {
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right;
  } else if (msg->buttons[6] == 1) {
    task_.is_under_ctrl = !task_.is_under_ctrl;
  }
}

// 启动不受对象管理的 detached 线程，然后把 Task 切到自动控制。
void EudmPlannerServer::Start() {
  std::thread(&EudmPlannerServer::MainThread, this).detach();
  task_.is_under_ctrl = true;
}

// 以 system_clock 固定周期运行；规划超时时下一轮会立即开始。
void EudmPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (rclcpp::ok()) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

// 清空输入队列只保留最新地图，运行规划并把行为写回地图后通知下游。
void EudmPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  bool has_updated_map = false;
  // 丢弃本周期前积压的旧快照，只保留最后一个 dequeue 结果。
  while (p_input_smm_buff_->try_dequeue(smm_)) {
    has_updated_map = true;
  }

  if (!has_updated_map) return;

  // manager 接收独立 shared_ptr 地图副本，避免后续队列更新修改本次输入。
  auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);

  // 把地图时间戳向下量化到 server 重规划周期网格。
  decimal_t replan_duration = 1.0 / work_rate_;
  double stamp = std::floor(smm_.time_stamp() / replan_duration) * replan_duration;

  if (bp_manager_.Run(stamp, map_ptr, task_) == kSuccess) {
    common::SemanticBehavior behavior;
    bp_manager_.ConstructBehavior(&behavior);
    smm_.set_ego_behavior(behavior);
  }

  // 即使 manager 失败也调用下游回调并发布可视化，地图行为可能保持输入旧值。
  if (has_callback_binded_) {
    private_callback_fn_(smm_);
  }

  PublishData();
}

// 保存下游回调并标记为已绑定；回调返回值当前不参与错误处理。
void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

// 直接写共享 Task 的用户期望速度。
void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

// 返回共享 Task 的用户期望速度。
decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
