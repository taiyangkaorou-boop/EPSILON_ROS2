/**
 * @file decoder.h
 * @brief ROS2 vehicle_msgs 到 common 内部车辆、Lane、障碍物和场景数据的头文件解码器。
 */

#ifndef _VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__
#define _VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__

#include <memory>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "vehicle_msgs/msg/arena_info.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"
#include "vehicle_msgs/msg/circle.hpp"
#include "vehicle_msgs/msg/circle_obstacle.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/msg/free_state.hpp"
#include "vehicle_msgs/msg/lane.hpp"
#include "vehicle_msgs/msg/lane_net.hpp"
#include "vehicle_msgs/msg/obstacle_set.hpp"
#include "vehicle_msgs/msg/occupancy_grid_float.hpp"
#include "vehicle_msgs/msg/occupancy_grid_u_int8.hpp"
#include "vehicle_msgs/msg/polygon_obstacle.hpp"
#include "vehicle_msgs/msg/state.hpp"
#include "vehicle_msgs/msg/vehicle.hpp"
#include "vehicle_msgs/msg/vehicle_param.hpp"
#include "vehicle_msgs/msg/vehicle_set.hpp"

#include "rclcpp/rclcpp.hpp"

namespace vehicle_msgs {

/**
 * @brief 以静态函数把 ROS2 消息写入调用方提供的内部数据对象。
 *
 * 集合级函数通常先 clear 再插入按 ID 索引的 map；基础几何函数部分采用 append。当前固定返回
 * 成功，不检查输出指针、header/frame 一致性、重复 ID 或数值/数组合法性。
 */
class Decoder {
 public:
  /// 解码二维自由状态，并把 ROS header stamp 转为秒。
  static ErrorType GetFreeStateMsgFromRosFreeState(
      const vehicle_msgs::msg::FreeState &in_state, common::FreeState *state) {
    state->time_stamp = rclcpp::Time(in_state.header.stamp).seconds();
    state->position[0] = in_state.pos.x;
    state->position[1] = in_state.pos.y;
    state->velocity[0] = in_state.vel.x;
    state->velocity[1] = in_state.vel.y;
    state->acceleration[0] = in_state.acc.x;
    state->acceleration[1] = in_state.acc.y;
    state->angle = in_state.angle;
    return kSuccess;
  }

  /// 解码规划状态，并以消息 header stamp 设置内部 time_stamp。
  static ErrorType GetStateFromRosStateMsg(const vehicle_msgs::msg::State &in_state,
                                           common::State *state) {
    state->time_stamp = rclcpp::Time(in_state.header.stamp).seconds();
    state->vec_position[0] = in_state.vec_position.x;
    state->vec_position[1] = in_state.vec_position.y;
    state->angle = in_state.angle;
    state->curvature = in_state.curvature;
    state->velocity = in_state.velocity;
    state->acceleration = in_state.acceleration;
    state->steer = in_state.steer;
    return kSuccess;
  }

  /// 清空 VehicleSet 后逐车解码；重复 ID 使用 insert 静默保留首项。
  static ErrorType GetVehicleSetFromRosVehicleSet(
      const vehicle_msgs::msg::VehicleSet &msg, common::VehicleSet *p_vehicle_set) {
    p_vehicle_set->vehicles.clear();
    for (int i = 0; i < (int)msg.vehicles.size(); ++i) {
      common::Vehicle vehicle;
      GetVehicleFromRosVehicle(msg.vehicles[i], &vehicle);
      p_vehicle_set->vehicles.insert(
          std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
    }
    return kSuccess;
  }

  /// 解码单车 ID/类别/类型、参数和状态。
  static ErrorType GetVehicleFromRosVehicle(const vehicle_msgs::msg::Vehicle &msg,
                                            common::Vehicle *p_vehicle) {
    p_vehicle->set_id(msg.id.data);
    p_vehicle->set_subclass(msg.subclass.data);
    p_vehicle->set_type(msg.type.data);
    common::VehicleParam param;
    GetVehicleParamFromRosVehicleParam(msg.param, &param);
    common::State state;
    GetStateFromRosStateMsg(msg.state, &state);
    p_vehicle->set_param(param);
    p_vehicle->set_state(state);
    return kSuccess;
  }

  /// 解码车辆尺寸、轴距、悬架和动力学限制。
  static ErrorType GetVehicleParamFromRosVehicleParam(
      const vehicle_msgs::msg::VehicleParam &msg,
      common::VehicleParam *p_vehicle_param) {
    p_vehicle_param->set_width(msg.width);
    p_vehicle_param->set_length(msg.length);
    p_vehicle_param->set_wheel_base(msg.wheel_base);
    p_vehicle_param->set_front_suspension(msg.front_suspension);
    p_vehicle_param->set_rear_suspension(msg.rear_suspension);
    p_vehicle_param->set_max_steering_angle(msg.max_steering_angle);
    p_vehicle_param->set_max_longitudinal_acc(msg.max_longitudinal_acc);
    p_vehicle_param->set_max_lateral_acc(msg.max_lateral_acc);
    p_vehicle_param->set_d_cr(msg.d_cr);
    return kSuccess;
  }

  /// 清空 LaneNet 后逐 Lane 解码；重复 Lane ID 静默保留首项。
  static ErrorType GetLaneNetFromRosLaneNet(const vehicle_msgs::msg::LaneNet &msg,
                                            common::LaneNet *p_lane_net) {
    p_lane_net->lane_set.clear();
    for (const auto &lane_msg : msg.lanes) {
      common::LaneRaw lane_raw;
      GetLaneRawFromRosLane(lane_msg, &lane_raw);
      p_lane_net->lane_set.insert(
          std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
    }
    return kSuccess;
  }

  /// 解码 Lane 拓扑、换道标志、行为、长度和中心线；lane_points 当前不预先清空。
  static ErrorType GetLaneRawFromRosLane(const vehicle_msgs::msg::Lane &msg,
                                         common::LaneRaw *p_lane) {
    p_lane->id = msg.id;
    p_lane->dir = msg.dir;

    p_lane->child_id = msg.child_id;
    p_lane->father_id = msg.father_id;
    p_lane->l_lane_id = msg.l_lane_id;
    p_lane->l_change_avbl = msg.l_change_avbl;
    p_lane->r_lane_id = msg.r_lane_id;
    p_lane->r_change_avbl = msg.r_change_avbl;
    p_lane->behavior = msg.behavior;
    p_lane->length = msg.length;

    p_lane->start_point(0) = msg.start_point.x;
    p_lane->start_point(1) = msg.start_point.y;
    p_lane->final_point(0) = msg.final_point.x;
    p_lane->final_point(1) = msg.final_point.y;
    for (const auto pt : msg.points) {
      p_lane->lane_points.push_back(Vec2f(pt.x, pt.y));
    }
    return kSuccess;
  }

  /// 清空两类障碍物集合后按 ID 解码圆/多边形障碍物。
  static ErrorType GetObstacleSetFromRosObstacleSet(
      const vehicle_msgs::msg::ObstacleSet &msg, common::ObstacleSet *obstacle_set) {
    obstacle_set->obs_circle.clear();
    obstacle_set->obs_polygon.clear();
    for (const auto &obs : msg.obs_circle) {
      common::CircleObstacle obs_temp;
      GetCircleObstacleFromRosCircleObstacle(obs, &obs_temp);
      obstacle_set->obs_circle.insert(
          std::pair<int, common::CircleObstacle>(obs_temp.id, obs_temp));
    }
    for (const auto &obs : msg.obs_polygon) {
      common::PolygonObstacle obs_temp;
      GetPolygonObstacleFromRosPolygonObstacle(obs, &obs_temp);
      obstacle_set->obs_polygon.insert(
          std::pair<int, common::PolygonObstacle>(obs_temp.id, obs_temp));
    }
    return kSuccess;
  }

  /// 解码圆障碍物 ID 和几何。
  static ErrorType GetCircleObstacleFromRosCircleObstacle(
      const vehicle_msgs::msg::CircleObstacle &msg, common::CircleObstacle *circle) {
    circle->id = msg.id;
    GetCircleFromRosCircle(msg.circle, &circle->circle);
    return kSuccess;
  }

  /// 解码多边形障碍物 ID 和几何。
  static ErrorType GetPolygonObstacleFromRosPolygonObstacle(
      const vehicle_msgs::msg::PolygonObstacle &msg, common::PolygonObstacle *poly) {
    poly->id = msg.id;
    GetPolygonFromRosPolygon(msg.polygon, &poly->polygon);
    return kSuccess;
  }

  /// 解码二维圆心和半径，忽略消息 z。
  static ErrorType GetCircleFromRosCircle(const vehicle_msgs::msg::Circle &msg,
                                          common::Circle *circle) {
    circle->center.x = msg.center.x;
    circle->center.y = msg.center.y;
    circle->radius = msg.radius;
    return kSuccess;
  }

  /// 把 Point32 追加到内部 Polygon；当前不清空原 points。
  static ErrorType GetPolygonFromRosPolygon(const geometry_msgs::msg::Polygon &msg,
                                            common::Polygon *poly) {
    for (const auto p : msg.points) {
      common::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      poly->points.push_back(pt);
    }
    return kSuccess;
  }

  /// 解码完整 ArenaInfo 的时间戳、LaneNet、VehicleSet 和 ObstacleSet。
  static ErrorType GetSimulatorDataFromRosArenaInfo(
      const vehicle_msgs::msg::ArenaInfo &msg, rclcpp::Time *time_stamp,
      common::LaneNet *lane_net, common::VehicleSet *vehicle_set,
      common::ObstacleSet *obstacle_set) {
    *time_stamp = msg.header.stamp;
    GetLaneNetFromRosLaneNet(msg.lane_net, lane_net);
    GetVehicleSetFromRosVehicleSet(msg.vehicle_set, vehicle_set);
    GetObstacleSetFromRosObstacleSet(msg.obstacle_set, obstacle_set);
    return kSuccess;
  }

  /// 解码静态 ArenaInfo 的时间戳、LaneNet 和 ObstacleSet。
  static ErrorType GetSimulatorDataFromRosArenaInfoStatic(
      const vehicle_msgs::msg::ArenaInfoStatic &msg, rclcpp::Time *time_stamp,
      common::LaneNet *lane_net, common::ObstacleSet *obstacle_set) {
    *time_stamp = msg.header.stamp;
    GetLaneNetFromRosLaneNet(msg.lane_net, lane_net);
    GetObstacleSetFromRosObstacleSet(msg.obstacle_set, obstacle_set);
    return kSuccess;
  }

  /// 解码动态 ArenaInfo 的时间戳和 VehicleSet。
  static ErrorType GetSimulatorDataFromRosArenaInfoDynamic(
      const vehicle_msgs::msg::ArenaInfoDynamic &msg, rclcpp::Time *time_stamp,
      common::VehicleSet *vehicle_set) {
    *time_stamp = msg.header.stamp;
    GetVehicleSetFromRosVehicleSet(msg.vehicle_set, vehicle_set);
    return kSuccess;
  }

  /// 解码加速度、转角速度、开环标志和期望状态。
  static ErrorType GetControlSignalFromRosControlSignal(
      const vehicle_msgs::msg::ControlSignal &msg,
      common::VehicleControlSignal *ctrl) {
    ctrl->acc = msg.acc;
    ctrl->steer_rate = msg.steer_rate;
    ctrl->is_openloop = msg.is_openloop.data;
    GetStateFromRosStateMsg(msg.state, &(ctrl->state));
    return kSuccess;
  }
};

}  // namespace vehicle_msgs

#endif  //_VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__
