#include "phy_simulator/visualizer.h"

#include <tf2_ros/transform_broadcaster.h>

namespace phy_simulator {

Visualizer::Visualizer(rclcpp::Node::SharedPtr node) : node_(node) {
  // 三个绝对 topic 固定在 phy_simulator_planning_node 命名空间下。
  vehicle_set_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/phy_simulator_planning_node/vis/vehicle_set_vis", 10);
  lane_net_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/phy_simulator_planning_node/vis/lane_net_vis", 10);
  obstacle_set_pub_ = 
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/phy_simulator_planning_node/vis/obstacle_set_vis", 10);
}

void Visualizer::VisualizeData() {
  // 读取 node 当前 ROS 时钟并转发到带时间戳版本。
  auto time_stamp = node_->get_clock()->now();
  VisualizeDataWithStamp(time_stamp);
  // TF 发布当前被关闭，且 SendTfWithStamp 尚无实现。
  // SendTfWithStamp(time_stamp);
}

void Visualizer::VisualizeDataWithStamp(const rclcpp::Time &stamp) {
  // PhySimulation getter 均按值返回，此处每帧复制三类完整数据。
  VisualizeVehicleSet(stamp, p_phy_sim_->vehicle_set());
  VisualizeLaneNet(stamp, p_phy_sim_->lane_net());
  VisualizeObstacleSet(stamp, p_phy_sim_->obstacle_set());
}

void Visualizer::VisualizeVehicleSet(const rclcpp::Time &stamp,
                                     const common::VehicleSet &vehicle_set) {
  visualization_msgs::msg::MarkerArray vehicle_marker;
  common::ColorARGB color_obb(1.0, 0.8, 0.8, 0.8);
  common::ColorARGB color_vel_vec(1.0, 1.0, 0.0, 0.0);
  common::ColorARGB color_steer(1.0, 1.0, 1.0, 1.0);
  // 每辆车由 VisualizationUtil 生成一组 Marker，起始 ID 使用 7*vehicle_id。
  for (auto iter = vehicle_set.vehicles.begin();
       iter != vehicle_set.vehicles.end(); ++iter) {
    common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(
        iter->second, common::ColorARGB(1, 1, 0, 0), color_vel_vec, color_steer,
        7 * iter->first, &vehicle_marker);
  }
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &vehicle_marker);
  vehicle_set_pub_->publish(vehicle_marker);
}

void Visualizer::VisualizeLaneNet(const rclcpp::Time &stamp,
                                  const common::LaneNet &lane_net) {
  visualization_msgs::msg::MarkerArray lane_net_marker;
  int id_cnt = 0;
  // unordered LaneSet 的每条 Lane 生成中心线、起点、终点和 ID 文本四个 Marker。
  for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
       ++iter) {
    visualization_msgs::msg::Marker lane_marker;
    common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVec(
        iter->second.lane_points, common::cmap.at("sky blue"),
        Vec3f(0.1, 0.1, 0.1), iter->second.id, &lane_marker);
    lane_marker.header.stamp = stamp;
    lane_marker.header.frame_id = "map";
    lane_marker.id = id_cnt++;
    lane_net_marker.markers.push_back(lane_marker);
    visualization_msgs::msg::Marker start_point_marker, end_point_marker,
        lane_id_text_marker;
    {
      // lane_points 必须非空，起点同时用于 ID 文本位置。
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
      // 终点取中心线最后一个采样点。
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
  lane_net_pub_->publish(lane_net_marker);
}

void Visualizer::VisualizeObstacleSet(const rclcpp::Time &stamp,
                                       const common::ObstacleSet &Obstacle_set) {
  // 公共工具把全部 PolygonObstacle 转为 MarkerArray，再统一填时间戳。
  visualization_msgs::msg::MarkerArray Obstacles_marker;
  common::VisualizationUtil::GetRosMarkerUsingObstacleSet(Obstacle_set,
                                                          &Obstacles_marker);
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &Obstacles_marker);
  obstacle_set_pub_->publish(Obstacles_marker);
}

}  // namespace phy_simulator
