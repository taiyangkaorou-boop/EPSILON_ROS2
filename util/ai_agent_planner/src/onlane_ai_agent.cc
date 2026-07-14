/**
 * @file onlane_ai_agent.cc
 * @brief 用 MPDM 行为规划和 on-lane 单步传播驱动仿真交通参与者的 ROS2 可执行入口。
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include "behavior_planner/behavior_server_ros.h"
#include "common/basics/tic_toc.h"
#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "sensor_msgs/msg/joy.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/encoder.h"

DECLARE_BACKWARD;
// 主控制、可视化和行为规划频率。
double fs_work_rate = 50.0;
double visualization_msg_rate = 20.0;
double bp_work_rate = 20.0;
int ego_id;

// 控制输出、行为规划 server、跨线程地图队列和语义地图可视化器。
rclcpp::Publisher<vehicle_msgs::msg::ControlSignal>::SharedPtr ctrl_signal_pub_;
std::shared_ptr<planning::BehaviorPlannerServer> p_bp_server_{nullptr};
std::shared_ptr<moodycamel::ReaderWriterQueue<semantic_map_manager::SemanticMapManager>> p_ctrl_input_smm_buff_{nullptr};
std::shared_ptr<semantic_map_manager::Visualizer> p_smm_vis_{nullptr};

// 控制器内部滚动目标状态、初始化标志和用户期望速度。
common::State desired_state;
bool has_init_state = false;
double desired_vel;

// 可视化调度、最新带行为地图和单步传播参数。
rclcpp::Time next_vis_pub_time;
semantic_map_manager::SemanticMapManager last_smm;
planning::OnLaneForwardSimulation::Param sim_param;

// 预留随机速度扰动状态；RandomBehavior 当前没有调用点。
std::mt19937 rng;
double vel_noise = 0.0;
int cnt = 0;
int aggressiveness_level = 3;

// 原始地图回调：送入 BehaviorPlanner，并用首帧真实自车状态初始化控制目标。
int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) {
    p_bp_server_->PushSemanticMap(smm);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_bp_server_ is nullptr in SemanticMapUpdateCallback");
  }

  if (!has_init_state) {
    desired_state = smm.ego_vehicle().state();
    has_init_state = true;
    desired_state.print();
  }
  return 0;
}

// 行为规划回调：把带 ego behavior 的地图写入控制线程 SPSC 队列。
int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ctrl_input_smm_buff_) {
    p_ctrl_input_smm_buff_->try_enqueue(smm);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_ctrl_input_smm_buff_ is nullptr in BehaviorUpdateCallback");
  }
  return 0;
}

// 每 2000 次调用采样 [-2,5] m/s 扰动并更新行为规划期望速度。
void RandomBehavior() {
  if (cnt == 0) {
    std::uniform_real_distribution<double> dist_vel(-2, 5);
    vel_noise = dist_vel(rng);
    if (p_bp_server_) {
      p_bp_server_->set_user_desired_velocity(desired_vel + vel_noise);
      RCLCPP_INFO(rclcpp::get_logger("onlane_ai_agent"), "[OnlaneAi]%d - desired velocity: %lf", ego_id, desired_vel + vel_noise);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_bp_server_ is nullptr in RandomBehavior");
    }
  }
  cnt++;
  if (cnt >= 2000) cnt = 0;
}

// 消费最新行为地图，传播内部目标状态一步并发布 ControlSignal/低频可视化。
void PublishControl() {
  if (!has_init_state) return;
  if (p_bp_server_ == nullptr) return;
  if (p_ctrl_input_smm_buff_ == nullptr) return;

  // 清空队列只保留最新地图，并用连续地图时间戳计算传播步长。
  bool is_map_updated = false;
  decimal_t previous_stamp = last_smm.time_stamp();
  while (p_ctrl_input_smm_buff_->try_dequeue(last_smm)) {
    is_map_updated = true;
  }
  if (!is_map_updated) return;

  decimal_t delta_t = last_smm.time_stamp() - previous_stamp;
  // 超过 100 个控制周期的间隔统一回退为一个 50 Hz 周期。
  if (delta_t > 100.0 / fs_work_rate) delta_t = 1.0 / fs_work_rate;

  // 行为规划参考速度再受当前参考 Lane 限速约束。
  decimal_t command_vel = p_bp_server_->reference_desired_velocity();
  decimal_t speed_limit;
  if (last_smm.GetSpeedLimit(last_smm.ego_vehicle().state(),
                             last_smm.ego_behavior().ref_lane,
                             &speed_limit) == kSuccess) {
    command_vel = std::min(speed_limit, command_vel);
  }
  common::Vehicle ego_vehicle = last_smm.ego_vehicle();
  // 几何/尺寸取最新地图，自车状态使用内部 desired_state 开环滚动值。
  ego_vehicle.set_state(desired_state);
  sim_param.idm_param.kDesiredVelocity = command_vel;

  common::Vehicle leading_vehicle;
  common::State state;
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = 2.2;
  // 未找到前车时保留默认 invalid Vehicle，交由传播器处理自由行驶。
  last_smm.GetLeadingVehicleOnLane(last_smm.ego_behavior().ref_lane,
                                   desired_state,
                                   last_smm.surrounding_vehicles(), lat_range,
                                   &leading_vehicle, &distance_residual_ratio);
  if (planning::OnLaneForwardSimulation::PropagateOnce(
          common::StateTransformer(last_smm.ego_behavior().ref_lane),
          ego_vehicle, leading_vehicle, delta_t, sim_param,
          &state) != kSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("onlane_ai_agent"), "[AiAgent]Err-Simulation error (with leading vehicle).");
    return;
  }

  common::VehicleControlSignal ctrl(state);
  // 把下一预测状态编码为 map frame 的 ROS2 控制消息。
  {
    vehicle_msgs::msg::ControlSignal ctrl_msg;
    vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
        ctrl, rclcpp::Clock(RCL_ROS_TIME).now(), std::string("map"), &ctrl_msg);
    ctrl_signal_pub_->publish(ctrl_msg);
  }
  desired_state = ctrl.state;

  // 按 visualization_msg_rate 发布语义地图 Marker 和 TF。
  {
    rclcpp::Time tnow = rclcpp::Clock(RCL_ROS_TIME).now();
    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += rclcpp::Duration::from_seconds(1.0 / visualization_msg_rate);
      if (p_smm_vis_) {
        p_smm_vis_->VisualizeDataWithStamp(tnow, last_smm);
        p_smm_vis_->SendTfWithStamp(tnow, last_smm);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_smm_vis_ is nullptr in PublishControl");
      }
    }
  }
}

// ROS2 入口：读取 agent 参数，组装 SMM/MPDM/控制链并以 50 Hz spin+control。
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("onlane_ai_agent");

  // 首次可视化允许在节点当前 ROS 时间立即发布。
  next_vis_pub_time = node->get_clock()->now();  


  double desired_vel = 6.0;
  int autonomous_level = 3;
  int aggressiveness_level = 3;
  std::string agent_config_path = "";

  // 声明并读取 ego、场景配置、速度、自动驾驶等级和激进程度参数。
  node->declare_parameter<int>("ego_id", ego_id);
  node->declare_parameter<std::string>("agent_config_path", agent_config_path);
  node->declare_parameter<double>("desired_vel", desired_vel);
  // 当前误用 aggressiveness_level 变量作为 autonomous_level 默认值。
  node->declare_parameter<int>("autonomous_level", aggressiveness_level);
  node->declare_parameter<int>("aggressiveness_level", aggressiveness_level);

  node->get_parameter("ego_id", ego_id);
  node->get_parameter("desired_vel", desired_vel);
  node->get_parameter("agent_config_path", agent_config_path);
  node->get_parameter("autonomous_level", autonomous_level);
  node->get_parameter("aggressiveness_level", aggressiveness_level);

  // 再次读取参数用于逐项日志和错误诊断。
  if (!node->get_parameter("ego_id", ego_id)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: ego_id");
  } else {
    RCLCPP_INFO(node->get_logger(), "ego_id: %d", ego_id);
  }

  if (!node->get_parameter("desired_vel", desired_vel)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: desired_vel");
  } else {
    RCLCPP_INFO(node->get_logger(), "desired_vel: %f", desired_vel);
  }
  
  if (!node->get_parameter("agent_config_path", agent_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: agent_config_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "agent_config_path: %s", agent_config_path.c_str());
  }

  if (!node->get_parameter("autonomous_level", autonomous_level)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: autonomous_level");
  } else {
    RCLCPP_INFO(node->get_logger(), "autonomous_level: %d", autonomous_level);
  }

  if (!node->get_parameter("aggressiveness_level", aggressiveness_level)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: aggressiveness_level");
  } else {
    RCLCPP_INFO(node->get_logger(), "aggressiveness_level: %d", aggressiveness_level);
  }


  try {
    // 每个 agent 发布到 launch remap 后的独立 ctrl topic。
    ctrl_signal_pub_ = node->create_publisher<vehicle_msgs::msg::ControlSignal>("ctrl", 10);

    // 随机源使用当前高分辨率时间播种；当前 RandomBehavior 未进入主循环。
    rng.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());

    // 用 aggressiveness lookup 初始化单步前向传播参数。
    planning::MultiModalForward::ParamLookUp(aggressiveness_level, &sim_param);
    RCLCPP_INFO(node->get_logger(), "[OnlaneAi]%d - aggresive: %d", ego_id, aggressiveness_level);

    // 语义地图负责 ROS 输入解码和当前 agent 环境建模。
    auto semantic_map_manager = std::make_shared<semantic_map_manager::SemanticMapManager>(ego_id, agent_config_path);
    auto smm_ros_adapter = std::make_shared<semantic_map_manager::RosAdapter>(node, semantic_map_manager.get());
    p_smm_vis_ = std::make_shared<semantic_map_manager::Visualizer>(node, ego_id);

    // MPDM BehaviorPlanner 产生参考 Lane、行为和参考速度。
    p_bp_server_ = std::make_shared<planning::BehaviorPlannerServer>(node, bp_work_rate, ego_id);
    if (p_bp_server_ == nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Failed to create BehaviorPlannerServer");
      return -1;
    } else {
      RCLCPP_INFO(node->get_logger(), "Success to create BehaviorPlannerServer");
    }

    p_bp_server_->set_user_desired_velocity(desired_vel);
    p_bp_server_->set_autonomous_level(autonomous_level);
    p_bp_server_->set_aggressive_level(aggressiveness_level);
    p_bp_server_->enable_hmi_interface();

    // 建立原始地图 -> 行为规划 -> 控制队列的两级回调链。
    smm_ros_adapter->BindMapUpdateCallback(SemanticMapUpdateCallback);
    p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);

    smm_ros_adapter->Init();
    p_bp_server_->Init();

    p_ctrl_input_smm_buff_ = std::make_shared<moodycamel::ReaderWriterQueue<semantic_map_manager::SemanticMapManager>>(100);

    // BehaviorPlanner 使用内部线程；主线程负责 ROS 回调和控制传播。
    p_bp_server_->Start();
    rclcpp::Rate rate(fs_work_rate);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      PublishControl();
      rate.sleep();
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
