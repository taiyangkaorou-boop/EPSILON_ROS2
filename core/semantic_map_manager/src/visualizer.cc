#include "semantic_map_manager/visualizer.h"

namespace semantic_map_manager {

Visualizer::Visualizer(rclcpp::Node::SharedPtr node, int node_id)
    : node_(node), node_id_(node_id), ego_to_map_tf_(node) {
  // 每个 agent 使用唯一 ego 可视化 TF 子 frame。
  ego_tf_name_ = "ego_vehicle_vis_" + std::to_string(node_id_);

  std::cout << "node_id_ = " << node_id_ << std::endl;
  std::cout << "ego_tf_name_ = " << ego_tf_name_ << std::endl;

  // 所有图层使用 /vis/agent_<id>/ 前缀，避免多 ego publisher 直接共用 topic。
  std::string ego_vehicle_vis_topic = std::string("/vis/agent_") +
                                      std::to_string(node_id_) +
                                      std::string("/ego_vehicle_vis");
  std::string obstacle_map_vis_topic = std::string("/vis/agent_") +
                                       std::to_string(node_id_) +
                                       std::string("/obstacle_map");
  std::string surrounding_lane_net_vis_topic =
      std::string("/vis/agent_") + std::to_string(node_id_) +
      std::string("/surrounding_lane_net_vis");
  std::string local_lanes_vis_topic = std::string("/vis/agent_") +
                                      std::to_string(node_id_) +
                                      std::string("/local_lanes_vis");
  std::string ego_vehicle_behavior_topic = std::string("/vis/agent_") +
                                           std::to_string(node_id_) +
                                           std::string("/ego_behavior_vis");
  std::string pred_intention_topic = std::string("/vis/agent_") +
                                     std::to_string(node_id_) +
                                     std::string("/pred_initial_intention_vis");
  std::string pred_traj_openloop_topic = std::string("/vis/agent_") +
                                         std::to_string(node_id_) +
                                         std::string("/pred_traj_openloop_vis");
  std::string surrounding_vehicle_topic =
      std::string("/vis/agent_") + std::to_string(node_id_) +
      std::string("/surrounding_vehicle_vis");
  std::string speed_limit_topic = std::string("/vis/agent_") +
                                  std::to_string(node_id_) +
                                  std::string("/speed_limit");
  // 每类可视化 publisher 的 QoS 队列深度均固定为 1，只保留最新消息。
  ego_vehicle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(ego_vehicle_vis_topic, 1);
  obstacle_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(obstacle_map_vis_topic, 1);
  surrounding_lane_net_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(surrounding_lane_net_vis_topic, 1);
  local_lanes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(local_lanes_vis_topic, 1);
  behavior_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(ego_vehicle_behavior_topic, 1);
  pred_traj_openloop_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(pred_traj_openloop_topic, 1);
  pred_intention_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(pred_intention_topic, 1);
  surrounding_vehicle_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(surrounding_vehicle_topic, 1);
  speed_limit_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(speed_limit_topic, 1);
}

void Visualizer::VisualizeData(const SemanticMapManager &smm) {
  // 未设置有效 SMM 时间戳时整帧可视化和 TF 都跳过。
  if (smm.time_stamp() < kEPS) return;
  // auto time_stamp = node_->get_clock()->now();
  // baseline 直接把 SMM 的 double 时间戳传入 rclcpp::Time 构造函数。
  auto time_stamp = rclcpp::Time(smm.time_stamp());  
  VisualizeDataWithStamp(time_stamp, smm);
  SendTfWithStamp(time_stamp, smm);
}

void Visualizer::VisualizeDataWithStamp(const rclcpp::Time &stamp,
                                        const SemanticMapManager &smm) {
  // 固定顺序发布自车、占据图、Lane、行为、预测周车和限速九类图层。
  VisualizeEgoVehicle(stamp, smm.ego_vehicle());
  VisualizeObstacleMap(stamp, smm.obstacle_map());
  VisualizeSurroundingLaneNet(stamp, smm.surrounding_lane_net(),
                              std::vector<int>());
  VisualizeLocalLanes(stamp, smm.local_lanes(), smm, std::vector<int>());
  VisualizeBehavior(stamp, smm.ego_behavior());
  VisualizeIntentionPrediction(stamp, smm.semantic_surrounding_vehicles());
  VisualizeOpenloopTrajPrediction(stamp, smm.openloop_pred_trajs());
  VisualizeSurroundingVehicles(stamp, smm.surrounding_vehicles(),
                               smm.key_vehicle_ids());
  VisualizeSpeedLimit(stamp, smm.RetTrafficInfoSpeedLimit());
}

void Visualizer::VisualizeDataWithStampForPlayback(
    const rclcpp::Time &stamp, const SemanticMapManager &smm,
    const std::vector<int> &deleted_lane_ids) {
  // 播放路径仅把 deleted_lane_ids 传给原始/本地 Lane 图层，其余图层与实时路径相同。
  VisualizeEgoVehicle(stamp, smm.ego_vehicle());
  VisualizeObstacleMap(stamp, smm.obstacle_map());
  VisualizeSurroundingLaneNet(stamp, smm.surrounding_lane_net(),
                              deleted_lane_ids);
  VisualizeLocalLanes(stamp, smm.local_lanes(), smm, deleted_lane_ids);
  VisualizeBehavior(stamp, smm.ego_behavior());
  VisualizeIntentionPrediction(stamp, smm.semantic_surrounding_vehicles());
  VisualizeOpenloopTrajPrediction(stamp, smm.openloop_pred_trajs());
  VisualizeSurroundingVehicles(stamp, smm.surrounding_vehicles(),
                               smm.key_vehicle_ids());
  VisualizeSpeedLimit(stamp, smm.RetTrafficInfoSpeedLimit());
}

void Visualizer::VisualizeSurroundingVehicles(
    const rclcpp::Time &stamp, const common::VehicleSet &vehicle_set,
    const std::vector<int> &nearby_ids) {
  visualization_msgs::msg::MarkerArray vehicle_marker_list;
  for (const auto &v : vehicle_set.vehicles) {
    // 每辆周车转换为 OBB、速度和转向等一组 Marker，默认使用半透明紫色系。
    visualization_msgs::msg::MarkerArray vehicle_marker;
    common::ColorARGB color_obb(0.5, 0.2, 0.7, 1.0);
    common::ColorARGB color_vel_vec(0.4, 0.0, 1.0, 1.0);
    common::ColorARGB color_steer(0.4, 1.0, 1.0, 1.0);
    if (v.second.type().compare("brokencar") == 0) {
      // brokencar 使用单独颜色强调异常/静止车辆类型。
      color_obb = common::ColorARGB(0.4, 1.0, 0.2, 0.2);
    }
    // key_vehicle_ids 中的车辆提高三类 Marker 的 alpha。
    if (nearby_ids.end() !=
        std::find(nearby_ids.begin(), nearby_ids.end(), v.second.id())) {
      color_obb.a = 0.9;
      color_vel_vec.a = 0.9;
      color_steer.a = 0.9;
    }
    common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(
        v.second, color_obb, color_vel_vec, color_steer, 1, &vehicle_marker);
    for (auto &marker : vehicle_marker.markers)
      vehicle_marker_list.markers.push_back(marker);
  }
  // 重新编号并删除上一帧多出的车辆 Marker。
  int num_markers = static_cast<int>(vehicle_marker_list.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_surrounding_vehicle_marker_cnt_,
      &vehicle_marker_list);
  last_surrounding_vehicle_marker_cnt_ = num_markers;
  surrounding_vehicle_vis_pub_->publish(vehicle_marker_list);
}

void Visualizer::VisualizeEgoVehicle(const rclcpp::Time &stamp,
                                     const common::Vehicle &vehicle) {
  // 自车使用高不透明度配色生成 OBB、速度向量和转向 Marker。
  visualization_msgs::msg::MarkerArray vehicle_marker;
  common::ColorARGB color_obb(1.0, 0.66, 0.66, 0.66);
  common::ColorARGB color_vel_vec(1.0, 0.0, 1.0, 1.0);
  common::ColorARGB color_steer(1.0, 1.0, 1.0, 1.0);
  common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(
      vehicle, color_obb, color_vel_vec, color_steer, 1, &vehicle_marker);
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &vehicle_marker);
  ego_vehicle_pub_->publish(vehicle_marker);
}

void Visualizer::VisualizeObstacleMap(
    const rclcpp::Time &stamp,
    const common::GridMapND<ObstacleMapType, 2> &obstacle_map) {
  // 空 GridMap 不发布，也不会主动清除 RViz 中上一张 OccupancyGrid。
  if (obstacle_map.data_size() < 1) return;
  nav_msgs::msg::OccupancyGrid occ_map;
  common::VisualizationUtil::GetRosOccupancyGridUsingGripMap2D(
      obstacle_map, node_->get_clock()->now(), &occ_map);
  // 转换工具先使用当前 ROS clock，本层再覆盖为调用方传入时间戳。
  occ_map.header.stamp = stamp;
  obstacle_map_pub_->publish(occ_map);
}

void Visualizer::SendTfWithStamp(const rclcpp::Time &stamp,
                                 const SemanticMapManager &smm) {
  // 即使调用方给出有效 stamp，SMM 自身时间戳未设置时仍拒绝发送 TF。
  if (smm.time_stamp() < kEPS) {
    // printf("[Error]SMM timestamp is unset\n");
    return;
  }
  // 从自车 3DoF 状态构造 map -> ego_vehicle_vis_<id> 的刚体变换。
  Vec3f state = smm.ego_vehicle().Ret3DofState();
  geometry_msgs::msg::Pose pose;
  common::VisualizationUtil::GetRosPoseFrom3DofState(state, &pose);
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = stamp;
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = ego_tf_name_.c_str();
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation = pose.orientation;

  // ego_to_map_tf_.sendTransform(transformStamped);
  // 当前未使用成员 ego_to_map_tf_，而由函数内静态 broadcaster 发送所有实例的 TF。
  static tf2_ros::TransformBroadcaster br(node_);
  br.sendTransform(transformStamped);
}

void Visualizer::VisualizeSurroundingLaneNet(
    const rclcpp::Time &stamp, const common::LaneNet &lane_net,
    const std::vector<int> &deleted_lane_ids) {
  visualization_msgs::msg::MarkerArray lane_net_marker;
  int id_cnt = 0;
  for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
       ++iter) {
    // 播放删除列表中的原始 Lane 完全跳过，不生成折线、端点或文本。
    if (deleted_lane_ids.end() != std::find(deleted_lane_ids.begin(),
                                            deleted_lane_ids.end(),
                                            iter->second.id)) {
      continue;
    }
    visualization_msgs::msg::Marker lane_marker;
    // 每条 LaneRaw 先生成天蓝色中心线折线。
    common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVec(
        iter->second.lane_points, common::cmap.at("sky blue"),
        Vec3f(0.1, 0.1, 0.1), iter->second.id, &lane_marker);
    lane_marker.header.stamp = stamp;
    lane_marker.header.frame_id = "map";
    lane_marker.id = id_cnt++;
    lane_net_marker.markers.push_back(lane_marker);
    // 再用首点/尾点球体和首点上方文本标注 Lane ID。
    visualization_msgs::msg::Marker start_point_marker, end_point_marker,
        lane_id_text_marker;
    {
      start_point_marker.header.stamp = stamp;
      start_point_marker.header.frame_id = "map";
      Vec2f pt = *(iter->second.lane_points.begin());
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
          Vec3f(0.5, 0.5, 0.5), id_cnt++, &start_point_marker);
      lane_id_text_marker.header.stamp = stamp;
      lane_id_text_marker.header.frame_id = "map";
      common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
          Vec3f(pt(0), pt(1), 0.5), std::to_string(iter->second.id),
          common::ColorARGB(1.0, 0.0, 0.0, 1.0), Vec3f(0.6, 0.6, 0.6), id_cnt++,
          &lane_id_text_marker);
    }
    {
      end_point_marker.header.stamp = stamp;
      end_point_marker.header.frame_id = "map";
      Vec2f pt = *(iter->second.lane_points.rbegin());
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
          Vec3f(0.5, 0.5, 0.5), id_cnt++, &end_point_marker);
    }

    lane_net_marker.markers.push_back(start_point_marker);
    lane_net_marker.markers.push_back(end_point_marker);
    lane_net_marker.markers.push_back(lane_id_text_marker);
  }
  // 每条有效 Lane 产生四个 Marker，并通过上一帧计数清除残留 ID。
  int num_markers = static_cast<int>(lane_net_marker.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_surrounding_lanes_cnt_,
      &lane_net_marker);
  last_surrounding_lanes_cnt_ = num_markers;
  surrounding_lane_net_pub_->publish(lane_net_marker);
}

void Visualizer::VisualizeLocalLanes(
    const rclcpp::Time &stamp,
    const std::unordered_map<int, common::Lane> &local_lanes,
    const SemanticMapManager &smm, const std::vector<int> &deleted_lane_ids) {
  // 函数静态计数被所有 Visualizer 实例共享，用于本地 Lane Marker 删除。
  static int last_mks_num = 0;
  visualization_msgs::msg::MarkerArray mks;
  for (const auto &p_lane : local_lanes) {
    // 若本地拟合 Lane 包含任一已删除原始 Lane，则播放时跳过该本地 Lane。
    bool is_to_del = false;
    for (const auto &del_id : deleted_lane_ids) {
      if (smm.IsLocalLaneContainsLane(p_lane.first, del_id)) {
        is_to_del = true;
        break;
      }
    }
    if (is_to_del) continue;

    // 本地 Lane 以半透明洋红面片显示，并下沉 0.3 m 避免遮挡其他图层。
    visualization_msgs::msg::Marker mk;
    common::VisualizationUtil::GetMarkerByLane(
        p_lane.second, 1.0, Vec3f(1.0, 0.0, 0.0),
        common::cmap.at("magenta").set_a(0.2), -0.3, &mk);
    mks.markers.push_back(mk);
  }
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_mks_num, &mks);
  last_mks_num = mks.markers.size();
  local_lanes_pub_->publish(mks);
}

void Visualizer::VisualizeBehavior(const rclcpp::Time &stamp,
                                   const common::SemanticBehavior &behavior) {
  // 通用工具把 SemanticBehavior 的参考 Lane、自车/周车 rollout 等转换为 MarkerArray。
  visualization_msgs::msg::MarkerArray behavior_marker_arr;
  common::VisualizationUtil::GetRosMarkerArrUsingSemanticBehavior(
      behavior, &behavior_marker_arr);
  common::VisualizationUtil::FillStampInMarkerArray(stamp,
                                                    &behavior_marker_arr);

  // 再统一覆盖 map frame/ID，并删除上一帧多出的行为 Marker。
  int num_markers = static_cast<int>(behavior_marker_arr.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_behavior_marker_cnt_,
      &behavior_marker_arr);
  last_behavior_marker_cnt_ = num_markers;

  behavior_vis_pub_->publish(behavior_marker_arr);
}

void Visualizer::VisualizeSpeedLimit(
    const rclcpp::Time &stamp, const vec_E<common::SpeedLimit> &speed_limits) {
  visualization_msgs::msg::MarkerArray traffic_signal_arr;
  int id_cnt = 0;
  decimal_t offset_len = 0;
  for (int i = 0; i < static_cast<int>(speed_limits.size()); ++i) {
    // 正常限速显示 Speed limit/Release；最大速度近零时复用为 Red light/Forbidden。
    std::string str_start = std::string("Speed limit: ");
    std::string str_end = std::string("Release\n");

    common::ColorARGB start_marker_color =
        common::ColorARGB(1.0, 1.0, 1.0, 0.0);
    common::ColorARGB end_marker_color = common::ColorARGB(1.0, 0.0, 1.0, 0.0);
    if (speed_limits[i].vel_range()(1) < kEPS) {
      str_start = std::string("Red light: ");
      str_end = std::string("Forbidden\n");
      start_marker_color = common::ColorARGB(1.0, 1.0, 0.0, 0.0);
      end_marker_color = common::ColorARGB(1.0, 1.0, 0.0, 0.0);
    }

    // 起点六边形标牌按 start_angle 定向；offset_len 当前为零，不产生位置偏移。
    visualization_msgs::msg::Marker start_marker;
    start_marker.header.stamp = stamp;
    start_marker.header.frame_id = "map";
    decimal_t start_point_x = speed_limits[i].start_point()(0);
    decimal_t start_point_y = speed_limits[i].start_point()(1);
    decimal_t start_point_angle = speed_limits[i].start_angle();

    decimal_t start_x = start_point_x - offset_len * cos(start_point_angle);
    decimal_t start_y = start_point_y - offset_len * sin(start_point_angle);

    common::VisualizationUtil::GetRosMarkerMeshHexagonSignUsingPosition(
        Vec3f(start_x, start_y, start_point_angle), 1.0, start_marker_color,
        ++id_cnt, &start_marker);
    traffic_signal_arr.markers.push_back(start_marker);

    // 起点上方文本显示最大速度，零限速仍会追加数值和 m/s 单位。
    visualization_msgs::msg::Marker start_text_marker;
    start_text_marker.header.stamp = stamp;
    start_text_marker.header.frame_id = "map";
    str_start += std::string(common::GetStringByValueWithPrecision<decimal_t>(
                                 speed_limits[i].vel_range()(1), 1) +
                             "m/s");
    common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
        Vec3f(start_x, start_y, 3.5), str_start, common::cmap.at("black"),
        Vec3f(0.75, 0.75, 0.75), ++id_cnt, &start_text_marker);
    traffic_signal_arr.markers.push_back(start_text_marker);

    // 终点采用 end_angle 定向，并发布解除限速或禁行提示。
    visualization_msgs::msg::Marker end_marker;
    end_marker.header.stamp = stamp;
    end_marker.header.frame_id = "map";
    decimal_t end_point_x = speed_limits[i].end_point()(0);
    decimal_t end_point_y = speed_limits[i].end_point()(1);
    decimal_t end_point_angle = speed_limits[i].end_angle();

    decimal_t end_x = end_point_x - offset_len * cos(end_point_angle);
    decimal_t end_y = end_point_y - offset_len * sin(end_point_angle);

    common::VisualizationUtil::GetRosMarkerMeshHexagonSignUsingPosition(
        Vec3f(end_x, end_y, speed_limits[i].end_angle()), 1.0, end_marker_color,
        ++id_cnt, &end_marker);
    traffic_signal_arr.markers.push_back(end_marker);

    visualization_msgs::msg::Marker end_text_marker;
    end_text_marker.header.stamp = stamp;
    end_text_marker.header.frame_id = "map";
    common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
        Vec3f(end_x, end_y, 3.5), str_end, common::cmap.at("black"),
        Vec3f(0.75, 0.75, 0.75), ++id_cnt, &end_text_marker);
    traffic_signal_arr.markers.push_back(end_text_marker);
  }
  // 每个限速区间固定生成起终点标牌和文本共四个 Marker。
  int num_markers = static_cast<int>(traffic_signal_arr.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_speed_limit_marker_cnt_,
      &traffic_signal_arr);
  last_speed_limit_marker_cnt_ = num_markers;
  speed_limit_vis_pub_->publish(traffic_signal_arr);
}

void Visualizer::VisualizeIntentionPrediction(
    const rclcpp::Time &stamp,
    const common::SemanticVehicleSet &semantic_vehicles) {
  visualization_msgs::msg::MarkerArray mks;
  int cnt = 0;
  for (const auto &p_sv : semantic_vehicles.semantic_vehicles) {
    // 复制单个 SemanticVehicle，并在其当前位置上方显示横向行为概率。
    auto semantic_vehicle = p_sv.second;
    common::State state = semantic_vehicle.vehicle.state();
    geometry_msgs::msg::Point pt0, pt1;
    decimal_t z = 2.0;
    for (const auto &entry : semantic_vehicle.probs_lat_behaviors.probs) {
      common::LateralBehavior beh = entry.first;
      decimal_t prob = entry.second;

      // 概率近零的模态不生成 Marker；箭头长度为 2*prob。
      if (prob < kEPS) continue;

      decimal_t angle_offset = 0.0;
      decimal_t length = prob * 2.0;

      if (beh == common::LateralBehavior::kLaneChangeRight) {
        // LCR/LCL 分别相对车头向右/左旋转 90 度，LK 沿车头方向。
        angle_offset = -kPi / 2.0;
      } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
        angle_offset = +kPi / 2.0;
      } else if (beh == common::LateralBehavior::kLaneKeeping) {
        angle_offset = 0.0;
      }
      pt0.x = state.vec_position(0);
      pt0.y = state.vec_position(1);
      pt0.z = z;

      pt1.x = state.vec_position(0) + cos(state.angle + angle_offset) * length;
      pt1.y = state.vec_position(1) + sin(state.angle + angle_offset) * length;
      pt1.z = z;

      visualization_msgs::msg::Marker behavior_mk;
      behavior_mk.id = cnt++;
      behavior_mk.type = visualization_msgs::msg::Marker::ARROW;
      behavior_mk.action = visualization_msgs::msg::Marker::MODIFY;
      behavior_mk.points.push_back(pt0);
      behavior_mk.points.push_back(pt1);
      behavior_mk.scale.x = 0.2;
      behavior_mk.scale.y = 0.5;
      // scale.z 未设置；全部模态使用相同黄色，未附行为文本或概率数值。
      common::VisualizationUtil::FillColorInMarker(common::cmap.at("yellow"),
                                                   &behavior_mk);
      mks.markers.push_back(behavior_mk);
    }
  }
  // 统一 map frame/连续 ID，并删除上一帧消失的概率模态箭头。
  int num_mks = static_cast<int>(mks.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_intention_marker_cnt_, &mks);
  last_intention_marker_cnt_ = num_mks;
  pred_intention_vis_pub_->publish(mks);
}

void Visualizer::VisualizeOpenloopTrajPrediction(
    const rclcpp::Time &stamp,
    const std::unordered_map<int, vec_E<common::State>> &openloop_pred_trajs) {
  visualization_msgs::msg::MarkerArray mks;
  // 每辆车的开环轨迹由逐状态圆柱和一条连接折线组成，全部使用天蓝色。
  for (const auto &p_traj : openloop_pred_trajs) {
    std::vector<common::Point> points;
    for (const auto &ps : p_traj.second) {
      common::Point pt(ps.vec_position(0), ps.vec_position(1));
      pt.z = 0.25;
      points.push_back(pt);
      visualization_msgs::msg::Marker point_marker;
      common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
          common::Point(pt), Vec3f(0.5, 0.5, 0.1), common::cmap.at("sky blue"),
          0, &point_marker);
      mks.markers.push_back(point_marker);
    }
    // 即使轨迹为空，也会为该车辆追加一条空 LineStrip Marker。
    visualization_msgs::msg::Marker line_marker;
    common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
        points, Vec3f(0.1, 0.1, 0.1), common::cmap.at("sky blue"), 0,
        &line_marker);
    mks.markers.push_back(line_marker);
  }
  // unordered_map 遍历顺序决定 Marker ID 次序，再利用上一帧计数清除多余 ID。
  int num_mks = static_cast<int>(mks.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_traj_list_marker_cnt_, &mks);
  last_traj_list_marker_cnt_ = num_mks;
  pred_traj_openloop_vis_pub_->publish(mks);
}

}  // namespace semantic_map_manager
