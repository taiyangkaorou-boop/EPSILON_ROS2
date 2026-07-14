#include "behavior_planner/behavior_server_ros.h"

namespace planning {

BehaviorPlannerServer::BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id)
  : node_(node), work_rate_(20.0), ego_id_(ego_id) {
    // 可视化器非拥有地引用本对象内的 bp_；输入队列容量使用 Config 默认值 100。
    p_visualizer_ = std::make_unique<BehaviorPlannerVisualizer>(node, &bp_, ego_id);
    p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
}

BehaviorPlannerServer::BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id)
  : node_(node), work_rate_(work_rate), ego_id_(ego_id) {
    // 自定义频率构造路径只替换 work_rate_，其余资源初始化与默认构造路径相同。
    p_visualizer_ = std::make_unique<BehaviorPlannerVisualizer>(node, &bp_, ego_id);
    p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
}

void BehaviorPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  // try_enqueue 的布尔结果未向调用方返回，队列满时该帧地图会被静默丢弃。
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

void BehaviorPlannerServer::PublishData() {
  // 可视化消息使用发布时刻的 ROS clock，而不是输入地图或轨迹状态时间戳。
  p_visualizer_->PublishDataWithStamp(node_->get_clock()->now());
}

void BehaviorPlannerServer::Init() {
  // 初始化行为规划核心；baseline 不检查 Init 返回码。
  bp_.Init("bp");
  // 只有 Init 当时处于 L2 以上才创建 /joy 订阅，后续等级变化不会补建订阅。
  if (bp_.autonomous_level() >= 2) {
    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&BehaviorPlannerServer::JoyCallback, this, std::placeholders::_1));
  }
  bool use_sim_state = true;
  // node_->declare_parameter("use_sim_state", use_sim_state);
  // 从节点读取状态源开关；参数声明行在 baseline 中被注释。
  node_->get_parameter("use_sim_state", use_sim_state);
  bp_.set_use_sim_state(use_sim_state);
  p_visualizer_->Init();
}

void BehaviorPlannerServer::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  // 自动驾驶等级和显式 HMI 开关共同控制 Joy 指令是否生效。
  if (bp_.autonomous_level() < 2) return;
  if (!is_hmi_enabled_) return;

  int msg_id;
  if (msg->header.frame_id.empty()) {
    msg_id = 0;
  } else {
    // frame_id 被复用为十进制 ego ID；非数字字符串会由 std::stoi 抛出异常。
    msg_id = std::stoi(msg->header.frame_id);
  }

  if (msg_id != ego_id_) return;
  // 按键映射：2 左换道、1 右换道、3 加速 1 m/s、0 减速 1 m/s。
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 && msg->buttons[3] == 0)
    return;

  // else-if 固定了多键同时按下时的优先级：左换道、右换道、加速、减速。
  if (msg->buttons[2] == 1) {
    bp_.set_hmi_behavior(common::LateralBehavior::kLaneChangeLeft);
  } else if (msg->buttons[1] == 1) {
    bp_.set_hmi_behavior(common::LateralBehavior::kLaneChangeRight);
  } else if (msg->buttons[3] == 1) {
    bp_.set_user_desired_velocity(bp_.user_desired_velocity() + 1.0);
  } else if (msg->buttons[0] == 1) {
    bp_.set_user_desired_velocity(bp_.user_desired_velocity() - 1.0);
  }
}

void BehaviorPlannerServer::Start() {
  // BehaviorPlanner 保存 map_adapter_ 的非拥有裸指针，二者生命周期都隶属本 server。
  bp_.set_map_interface(&map_adapter_);
  // 后台线程 detach 后没有显式 stop/join；调用方必须保证 server 生命周期覆盖线程。
  std::thread(&BehaviorPlannerServer::MainThread, this).detach();
}

void BehaviorPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  // auto current_start_time = system_clock::now();
  // auto next_start_time = current_start_time;
  // 毫秒周期由 1000/work_rate_ 截断得到。
  const milliseconds interval(static_cast<int>(1000.0 / work_rate_));
  while (rclcpp::ok()) {
    // 每轮以实际开始时刻重新设定下一 deadline；规划超时不会补跑历史周期。
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

void BehaviorPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  // 清空当前积压队列，只保留最后一次出队的最新 SemanticMapManager 用于规划。
  SemanticMapManager smm;
  bool has_updated_map = false;
  while (p_input_smm_buff_->try_dequeue(smm)) {
    has_updated_map = true;
  }

  if (has_updated_map) {
    // 复制最新地图到 shared_ptr，并更新 BehaviorPlanner 使用的地图适配器快照。
    auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm);
    map_adapter_.set_map(map_ptr);

    TicToc timer;
    // 规划成功时才把新行为写回本地 smm；失败时仍继续回调和可视化流程。
    if (bp_.RunOnce() == kSuccess) {
      smm.set_ego_behavior(bp_.behavior());
    }

    if (has_callback_binded_) {
      // 回调在当前 detached 规划线程执行，返回值被忽略。
      private_callback_fn_(smm);
    }

    PublishData();
  }
}

void BehaviorPlannerServer::BindBehaviorUpdateCallback(std::function<int(const SemanticMapManager &)> fn) {
  // 保存一元回调包装并最后置启用标记；没有空函数或并发绑定检查。
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

void BehaviorPlannerServer::Replan() {}

void BehaviorPlannerServer::set_autonomous_level(int level) {
  // Server 不保留独立副本，配置直接转发给 BehaviorPlanner。
  bp_.set_autonomous_level(level);
}

void BehaviorPlannerServer::set_aggressive_level(int level) {
  // 激进程度将在 BehaviorPlanner 下一次 L3 RunOnce 中映射为传播参数。
  bp_.set_aggressive_level(level);
}

void BehaviorPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  // 是否接受和如何截断由 BehaviorPlanner 当前自动驾驶等级决定。
  bp_.set_user_desired_velocity(desired_vel);
}

decimal_t BehaviorPlannerServer::user_desired_velocity() const {
  return bp_.user_desired_velocity();
}

decimal_t BehaviorPlannerServer::reference_desired_velocity() const {
  return bp_.reference_desired_velocity();
}

void BehaviorPlannerServer::enable_hmi_interface() {
  // 只打开软件门控；Joy 订阅仍要求 Init 时自动驾驶等级不低于 L2。
  is_hmi_enabled_ = true;
}

}  // namespace planning
