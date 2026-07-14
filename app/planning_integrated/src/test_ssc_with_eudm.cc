/**
 * @file test_ssc_with_eudm.cc
 * @brief 组装 EUDM 行为决策、语义地图管理和 SSC 轨迹规划的 ROS2 集成节点。
 *
 * 本文件只负责系统编排：读取启动参数、连接上下游回调并管理各服务器生命周期。
 * 行为决策、地图解析和轨迹优化分别由 eudm_planner、semantic_map_manager 与
 * ssc_planner 实现，集成层不应复制这些模块内部的算法逻辑。
 */
#include "rclcpp/rclcpp.hpp"
#include <stdlib.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "eudm_planner/eudm_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
// 两个规划层均以 20 Hz 工作，对应 EPSILON 系统的 50 ms 在线规划周期。
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

// 服务器由 main 创建、由地图回调访问，因此在当前单节点进程内共享生命周期。
std::shared_ptr<planning::SscPlannerServer> p_ssc_server_{nullptr};
std::shared_ptr<planning::EudmPlannerServer> p_bp_server_{nullptr};

/**
 * @brief 将 EUDM 完成行为更新后的语义地图传递给 SSC 轨迹规划层。
 * @param smm 已包含行为层决策结果的语义地图快照。
 * @return 当前回调协议固定返回 0，表示消息已被接收。
 */
int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

/**
 * @brief 将感知/仿真侧更新的语义地图传递给 EUDM 行为决策层。
 * @param smm 由 RosAdapter 汇总的最新语义地图快照。
 * @return 当前回调协议固定返回 0，表示消息已被接收。
 */
int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

/**
 * @brief 创建并运行 EUDM + SSC 的 ROS2 集成进程。
 * @param argc ROS2 命令行参数数量。
 * @param argv ROS2 命令行参数数组。
 * @return 正常退出返回 0，初始化或运行异常返回 -1。
 */
int main(int argc, char** argv) {
  // 初始化 ROS2 上下文，并使用单节点承载地图、行为和运动规划服务器。
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_ssc_with_eudm");

  // 参数默认值仅用于声明；正常运行时由 launch 文件传入真实配置路径。
  int ego_id = 0;
  std::string agent_config_path = "";
  std::string bp_config_path = "";
  std::string ssc_config_path = "";
  double desired_vel = 6.0;

  node->declare_parameter<int>("ego_id", ego_id);
  node->declare_parameter<double>("desired_vel", desired_vel);
  node->declare_parameter<std::string>("agent_config_path", agent_config_path);
  node->declare_parameter<std::string>("bp_config_path", bp_config_path);
  node->declare_parameter<std::string>("ssc_config_path", ssc_config_path);

  // 逐项读取并打印参数，便于定位 launch 配置或包安装路径错误。
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

  if (!node->get_parameter("bp_config_path", bp_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: bp_config_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "bp_config_path: %s", bp_config_path.c_str());
  }

  if (!node->get_parameter("ssc_config_path", ssc_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: ssc_config_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "ssc_config_path: %s", ssc_config_path.c_str());
  }

  try {
    // 语义地图管理器是系统数据入口，RosAdapter 将 ROS2 话题转换为内部地图对象。
    auto semantic_map_manager = std::make_shared<semantic_map_manager::SemanticMapManager>(ego_id, agent_config_path);
    auto smm_ros_adapter = std::make_shared<semantic_map_manager::RosAdapter>(node, semantic_map_manager.get());
    smm_ros_adapter->BindMapUpdateCallback(SemanticMapUpdateCallback);

    // 行为层消费地图并输出带决策语义的地图；SSC 再据此生成连续轨迹。
    p_bp_server_ = std::make_shared<planning::EudmPlannerServer>(node, bp_work_rate, ego_id);
    p_bp_server_->set_user_desired_velocity(desired_vel);
    p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
    
    p_ssc_server_ = std::make_shared<planning::SscPlannerServer>(node, ssc_planner_work_rate, ego_id);

    // 先加载配置和建立订阅/发布关系，再启动各规划线程，避免使用未初始化资源。
    p_bp_server_->Init(bp_config_path);
    p_ssc_server_->Init(ssc_config_path);
    smm_ros_adapter->Init();

    p_bp_server_->Start();
    p_ssc_server_->Start();

    // 100 Hz 主循环只负责及时派发 ROS2 回调；规划频率由各服务器内部定时器控制。
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      rate.sleep();
    }

  } catch (const std::exception &e) {
    // 将跨模块初始化异常统一转换为 ROS2 错误日志并终止进程。
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    return -1;
  }

  // 正常停止 ROS2 上下文，使发布器、订阅器和后台线程按框架顺序释放。
  rclcpp::shutdown();
  return 0;
}
