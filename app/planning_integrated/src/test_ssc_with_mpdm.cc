/**
 * @file test_ssc_with_mpdm.cc
 * @brief 组装 MPDM 行为规划、语义地图管理和 SSC 轨迹规划的 ROS2 集成节点。
 *
 * 该入口与 EUDM 入口共享同一语义地图和 SSC 运动层，仅替换行为决策服务器，
 * 因而也是后续算法公平对比时保持下层规划器一致的重要系统边界。
 */
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "behavior_planner/behavior_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
// 行为层与运动层均以 20 Hz 更新，对应 50 ms 的规划周期。
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

// 全局共享指针用于连接语义地图回调与两个规划服务器。
std::shared_ptr<planning::SscPlannerServer> p_ssc_server_{nullptr};
std::shared_ptr<planning::BehaviorPlannerServer> p_bp_server_{nullptr};

/**
 * @brief 将 MPDM 行为规划结果传递给 SSC 轨迹规划服务器。
 * @param smm 已写入行为决策结果的语义地图快照。
 * @return 固定返回 0，符合现有回调接口约定。
 */
int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

/**
 * @brief 将最新语义地图传递给 MPDM 行为规划服务器。
 * @param smm 由 ROS2 静态/动态环境话题生成的地图快照。
 * @return 固定返回 0，符合现有回调接口约定。
 */
int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

/**
 * @brief 创建并运行 MPDM + SSC 的 ROS2 集成进程。
 * @param argc ROS2 命令行参数数量。
 * @param argv ROS2 命令行参数数组。
 * @return 正常退出返回 0，捕获异常时返回 -1。
 */
int main(int argc, char** argv) {
  // 初始化 ROS2，并以单节点承载系统级编排逻辑。
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_ssc_with_mpdm");

  // launch 文件应覆盖这些默认参数，尤其是场景和 SSC 配置路径。
  int ego_id = 0;
  std::string agent_config_path = "";
  std::string ssc_config_path = "";
  double desired_vel = 6.0;

  node->declare_parameter<int>("ego_id", ego_id);
  node->declare_parameter<double>("desired_vel", desired_vel);
  node->declare_parameter<std::string>("agent_config_path", agent_config_path);
  node->declare_parameter<std::string>("ssc_config_path", ssc_config_path);
  // 以下绝对路径仅是历史调试记录，不参与当前参数声明或运行逻辑。
  // node->declare_parameter<std::string>("agent_config_path", "/home/tao/Desktop/Autonomous-Motorsports-Motion-Planning-for-the-IAC/EPSILON/src/core/playgrounds/highway_v1.0/agent_config.json");
  // node->declare_parameter<std::string>("ssc_config_path", "/home/tao/Desktop/Autonomous-Motorsports-Motion-Planning-for-the-IAC/EPSILON/src/util/ssc_planner/config/ssc_config.pb.txt");

  // 逐项读取并记录参数，确保启动配置问题能够在节点日志中直接定位。
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

  if (!node->get_parameter("ssc_config_path", ssc_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: ssc_config_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "ssc_config_path: %s", ssc_config_path.c_str());
  }

  try {
    // 地图管理器负责内部语义地图，RosAdapter 负责 ROS2 消息与内部对象的转换。
    auto semantic_map_manager = std::make_shared<semantic_map_manager::SemanticMapManager>(ego_id, agent_config_path);
    auto smm_ros_adapter = std::make_shared<semantic_map_manager::RosAdapter>(node, semantic_map_manager.get());
    smm_ros_adapter->BindMapUpdateCallback(SemanticMapUpdateCallback);
    
    // MPDM 行为服务器输出离散行为，HMI 接口用于人工切换或调试行为状态。
    p_bp_server_ = std::make_shared<planning::BehaviorPlannerServer>(node, bp_work_rate, ego_id);
    p_bp_server_->set_user_desired_velocity(desired_vel);
    p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
    p_bp_server_->set_autonomous_level(3);
    p_bp_server_->enable_hmi_interface();

    // SSC 服务器复用 MPDM 输出的语义地图，负责连续安全轨迹生成。
    p_ssc_server_ = std::make_shared<planning::SscPlannerServer>(node, ssc_planner_work_rate, ego_id);

    // 完成各模块配置和 ROS2 接口初始化后，再启动内部工作线程。
    p_ssc_server_->Init(ssc_config_path);
    p_bp_server_->Init();
    smm_ros_adapter->Init();

    p_bp_server_->Start();
    p_ssc_server_->Start();

    // 主循环只派发 ROS2 回调，规划频率由服务器内部的 work_rate 控制。
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      rate.sleep();
    }

  } catch (const std::exception &e) {
    // 统一记录跨模块异常，避免节点在初始化失败后继续运行。
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    return -1;
  }

  // 请求 ROS2 框架有序关闭所有通信资源。
  rclcpp::shutdown();
  return 0;
}
