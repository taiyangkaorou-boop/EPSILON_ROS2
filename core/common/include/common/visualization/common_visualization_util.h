/**
 * @file common_visualization_util.h
 * @author HKUST Aerial Robotics Group
 * @brief common 几何、轨迹、语义对象到 ROS2 可视化消息的转换工具。
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__
#define _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__

#include "common/basics/basics.h"
#include "common/basics/colormap.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/basics/tool_func.h"
#include "common/circle_arc/circle_arc.h"
#include "common/circle_arc/circle_arc_branch.h"
#include "common/lane/lane.h"
#include "common/math/calculations.h"
#include "common/spline/polynomial.h"
#include "common/spline/spline.h"
#include "common/state/state.h"
#include "common/state/waypoint.h"
#include "common/trajectory/trajectory.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace common {

/**
 * @brief 无状态 ROS2 可视化消息构造与批量属性填充工具。
 *
 * 大多数接口采用追加语义，不清空 Marker 的 points/colors 或 MarkerArray；header、
 * frame_id、namespace 和时间戳通常由调用方或专用填充函数统一设置。本类不参与规划
 * 数值和碰撞判定。
 */
class VisualizationUtil {
 public:
  /**
   * @brief 在 `[s0,s1)` 上采样向量多项式并追加为 LINE_STRIP 点。
   * @param poly 输入向量多项式。
   * @param s0 参数起点。
   * @param s1 参数终点，不包含。
   * @param step 采样步长，baseline 要求为正。
   * @param scale Marker 三轴尺度。
   * @param color ARGB 颜色。
   * @param marker 输出 Marker；不设置 header、frame、namespace 或 id，也不清空旧点。
   */
  template <int N_DEG, int N_DIM>
  static ErrorType GetMarkerByPolynomial(const PolynomialND<N_DEG, N_DIM>& poly,
                                         const decimal_t s0, const decimal_t s1,
                                         const decimal_t step,
                                         const Vec3f scale,
                                         const ColorARGB color,
                                         visualization_msgs::msg::Marker* marker
) {
    // LINE_STRIP 使用 points 顺序连接采样位置。
    marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker->action = visualization_msgs::msg::Marker::MODIFY;
    FillScaleColorInMarker(scale, color, marker);
    for (decimal_t s = s0; s < s1; s += step) {
      // PolynomialND 当前没有返回值形式的 evaluate(s) 重载；模板实例化时会暴露错误。
      auto v = poly.evaluate(s);
      geometry_msgs::msg::Point point;
      ConvertVectorToPoint<N_DIM>(v, &point);
      marker->points.push_back(point);
    }
    return kSuccess;
  }

  /**
   * @brief 从样条起点到终点按固定步长采样并追加为 LINE_STRIP。
   * @param spline 输入分段样条。
   * @param step 正采样步长。
   * @param scale Marker 尺度。
   * @param color Marker 颜色。
   * @param offset_z 所有采样点使用的固定 z 偏移。
   * @param marker 输出 Marker，不清空已有点。
   */
  template <int N_DEG, int N_DIM>
  static ErrorType GetMarkerBySpline(const Spline<N_DEG, N_DIM>& spline,
                                     const decimal_t step, const Vec3f& scale,
                                     const ColorARGB& color,
                                     const decimal_t offset_z,
                                     visualization_msgs::msg::Marker* marker
) {
    marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker->action = visualization_msgs::msg::Marker::MODIFY;
    FillScaleColorInMarker(scale, color, marker);
    // 使用半开参数域，精确终点不会加入线条。
    for (decimal_t s = spline.begin(); s < spline.end(); s += step) {
      Vecf<N_DIM> ret;
      if (spline.evaluate(s, 0, &ret) == kSuccess) {
        geometry_msgs::msg::Point point;
        ConvertVectorToPoint<N_DIM>(ret, &point);
        point.z = offset_z;
        marker->points.push_back(point);
      }
    }

    return kSuccess;
  }

  /**
   * @brief 将有效 Lane 的位置样条解包后复用样条 LINE_STRIP 构造接口。
   * @return Lane 无效时返回 kIllegalInput，否则返回 kSuccess。
   */
  static ErrorType GetMarkerByLane(const Lane& lane, const decimal_t step,
                                   const Vec3f& scale, const ColorARGB& color,
                                   const decimal_t offset_z,
                                   visualization_msgs::msg::Marker* marker
) {
    if (!lane.IsValid()) return kIllegalInput;
    GetMarkerBySpline<LaneDegree, LaneDim>(lane.position_spline(), step, scale,
                                           color, offset_z, marker);
    return kSuccess;
  }

  /**
   * @brief 采样世界轨迹状态位置，构造一个 LINE_STRIP 并追加到 MarkerArray。
   * @param traj 输入轨迹。
   * @param step 正时间/参数步长。
   * @param scale 线条尺度。
   * @param color 线条颜色。
   * @param offset_z 固定高度偏移。
   * @param marker_arr 输出数组，不清空已有 Marker。
   */
  static ErrorType GetMarkerArrayByTrajectory(
      const Trajectory& traj, const decimal_t step, const Vec3f& scale,
      const ColorARGB& color, const decimal_t offset_z,
      visualization_msgs::msg::MarkerArray* marker_arr) {
    // 只检查轨迹有效标记，单个采样失败会被跳过而不中止整条可视化。
    if (!traj.IsValid()) return kIllegalInput;
    visualization_msgs::msg::Marker traj_mk;
    traj_mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj_mk.action = visualization_msgs::msg::Marker::MODIFY;
    FillScaleColorInMarker(scale, color, &traj_mk);
    // 终点不采样，step<=0 时可能无法结束。
    for (decimal_t s = traj.begin(); s < traj.end(); s += step) {
      common::State state;
      if (traj.GetState(s, &state) == kSuccess) {
        geometry_msgs::msg::Point point;
        point.x = state.vec_position[0];
        point.y = state.vec_position[1];
        point.z = offset_z;
        traj_mk.points.push_back(point);
      }
    }
    marker_arr->markers.push_back(traj_mk);
    return kSuccess;
  }

  /**
   * @brief 把固定维 double 向量的前三维复制到 ROS Point。
   * @note 缺失维度补零，三维以上分量忽略；当前参数按值复制输入向量。
   */
  template <int N_DIM>
  static ErrorType ConvertVectorToPoint(const Vecf<N_DIM> vec,
                                        geometry_msgs::msg::Point* point) {
    point->x = 0.0;
    point->y = 0.0;
    point->z = 0.0;

    // 使用编译期维数分支，只映射 x/y/z。
    if (N_DIM >= 1) {
      point->x = vec[0];
    }
    if (N_DIM >= 2) {
      point->y = vec[1];
    }
    if (N_DIM >= 3) {
      point->z = vec[2];
    }
    return kSuccess;
  }

  /**
   * @brief 把固定维向量前三维转换为 ROS Point32 单精度字段。
   * @note 缺失维度补零，额外维度忽略。
   */
  template <int N_DIM>
  static ErrorType ConvertVectorToPoint32(const Vecf<N_DIM>& vec,
                                          geometry_msgs::msg::Point32* point) {
    point->x = 0.0;
    point->y = 0.0;
    point->z = 0.0;

    if (N_DIM >= 1) {
      point->x = vec[0];
    }
    if (N_DIM >= 2) {
      point->y = vec[1];
    }
    if (N_DIM >= 3) {
      point->z = vec[2];
    }
    return kSuccess;
  }

  /**
   * @brief 把 State 序列的位置连接为固定线宽 LINE_STRIP 并追加到 MarkerArray。
   * @param state_vec 通常来自同一条轨迹的世界状态序列。
   * @param color 线条颜色。
   * @param marker_arr 输出数组，不清空旧 Marker。
   */
  static ErrorType GetMarkerArrayByStateVector(
      const vec_E<State>& state_vec, const ColorARGB& color,
      visualization_msgs::msg::MarkerArray* marker_arr) {
    visualization_msgs::msg::Marker traj_mk;
    traj_mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj_mk.action = visualization_msgs::msg::Marker::MODIFY;
    FillScaleColorInMarker(Vec3f(0.05, 0.05, 0.05), color, &traj_mk);
    for (auto& state : state_vec) {
      geometry_msgs::msg::Point point;
      ConvertVectorToPoint<2>(state.vec_position, &point);
      traj_mk.points.push_back(point);
    }
    marker_arr->markers.push_back(traj_mk);

    return kSuccess;
  }

  /**
   * @brief 把 Marker pose 重置为原点单位姿态，并填充尺度和颜色。
   * @note 若 Marker 已有位置/姿态，本函数会覆盖它们。
   */
  static ErrorType FillScaleColorInMarker(const Vec3f scale,
                                          const ColorARGB color,
                                          visualization_msgs::msg::Marker* marker
) {
    // 默认 pose 使用世界原点和单位四元数。
    marker->pose.position.x = 0.0;
    marker->pose.position.y = 0.0;
    marker->pose.position.z = 0.0;
    marker->pose.orientation.w = 1.0;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;

    FillColorInMarker(color, marker);
    FillScaleInMarker(scale, marker);
    return kSuccess;
  }

  /// 将 ColorARGB 的 a/r/g/b 分量复制到 ROS Marker 颜色。
  static ErrorType FillColorInMarker(const ColorARGB& color,
                                     visualization_msgs::msg::Marker* marker
) {
    marker->color.a = color.a;
    marker->color.r = color.r;
    marker->color.g = color.g;
    marker->color.b = color.b;
    return kSuccess;
  }

  /// 将三维尺度向量复制到 Marker scale.x/y/z。
  static ErrorType FillScaleInMarker(const Vec3f scale,
                                     visualization_msgs::msg::Marker* marker
) {
    marker->scale.x = scale(0);
    marker->scale.y = scale(1);
    marker->scale.z = scale(2);
    return kSuccess;
  }

  /**
   * @brief 按 points 顺序追加 Jet 渐变颜色。
   * @param if_ascending 当前实现未使用，颜色始终从低值向高值递增。
   * @param marker 输入点集及输出 colors；旧颜色不会清空。
   */
  static ErrorType FillGradientColorInMarker(
      const bool if_ascending, visualization_msgs::msg::Marker* marker
) {
    int num = marker->points.size();
    for (int i = 0; i < num; ++i) {
      // 使用 i/num，最后一个点不会达到色图上界 1.0。
      double k = (double)i / (double)num;
      common::ColorARGB c = common::GetJetColorByValue(k, 1.0, 0.0);
      std_msgs::msg::ColorRGBA c_ros;
      c_ros.a = c.a;
      c_ros.r = c.r;
      c_ros.g = c.g;
      c_ros.b = c.b;
      marker->colors.push_back(c_ros);
    }
    return kSuccess;
  }

  /**
   * @brief 为数组内 Marker 依次设置 id/header，并追加删除上一帧多余 id 的 Marker。
   * @param time_stamp 统一 ROS 时间戳。
   * @param frame_id 统一坐标系名称。
   * @param last_array_size 上一帧 Marker 数量。
   * @param marker_arr 当前帧数组；函数会在末尾追加 DELETE Marker。
   */
  static ErrorType FillHeaderIdInMarkerArray(
      const rclcpp::Time time_stamp, 
      const std::string frame_id,
      const int last_array_size, 
      visualization_msgs::msg::MarkerArray* marker_arr) {
    // 当前 Marker 按数组顺序从 0 重新编号，同时覆盖各自原 header。
    int marker_id = 0;
    for (auto& mk : marker_arr->markers) {
      mk.id = marker_id;
      mk.header.stamp = time_stamp;
      mk.header.frame_id = frame_id;
      marker_id++;
    }

    // 若上一帧更长，为缺失 id 追加删除消息；namespace 保持默认空字符串。
    visualization_msgs::msg::Marker delete_mk;
    delete_mk.header.stamp = time_stamp;
    delete_mk.header.frame_id = frame_id;
    delete_mk.action = visualization_msgs::msg::Marker::DELETE;
    for (int i = marker_id; i < last_array_size; i++) {
      delete_mk.id = i;
      marker_arr->markers.push_back(delete_mk);
    }
    return kSuccess;
  }

  /**
   * @brief 覆盖 MarkerArray 中所有 Marker 的 header.stamp，保留其他 header 字段。
   */
  static ErrorType FillStampInMarkerArray(
      const rclcpp::Time time_stamp, visualization_msgs::msg::MarkerArray* marker_arr) {
    for (auto& mk : marker_arr->markers) {
      mk.header.stamp = time_stamp;
    }
    return kSuccess;
  }

  /**
   * @brief 为 MarkerArray 中所有 Marker 设置统一生命周期。
   */
  static ErrorType FillLifeTimeInMarkerArray(
      const rclcpp::Duration duration,
      visualization_msgs::msg::MarkerArray* marker_arr) {
    for (auto& mk : marker_arr->markers) {
      mk.lifetime = duration;
    }
    return kSuccess;
  }

  /**
   * @brief 将 `[x,y,yaw]` 转换为 z=0 的 ROS Pose。
   * @param state 平面三自由度状态。
   * @param p_pose 输出位置和由 yaw 构造的单位四元数。
   */
  static ErrorType GetRosPoseFrom3DofState(const Vec3f& state,
                                           geometry_msgs::msg::Pose* p_pose) {
    p_pose->position.x = state(0);
    p_pose->position.y = state(1);
    p_pose->position.z = 0.0;

    // roll/pitch 固定为零，仅编码平面 yaw。
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state(2));

    p_pose->orientation.x = q.x();
    p_pose->orientation.y = q.y();
    p_pose->orientation.z = q.z();
    p_pose->orientation.w = q.w();
    return kSuccess;
  }

  /**
   * @brief 构造 map 坐标系、当前 ROS 时间戳的 PoseStamped。
   * @note 时间戳来自函数内临时 RCL_ROS_TIME Clock，而非调用节点显式传入的时钟。
   */
  static ErrorType GetRosPoseStampedFromVec3d(
      const Vec3f& state, geometry_msgs::msg::PoseStamped* p_pose_stamped) {
    p_pose_stamped->header.frame_id = "map";
    p_pose_stamped->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    geometry_msgs::msg::Pose pose;
    GetRosPoseFrom3DofState(state, &pose);
    p_pose_stamped->pose = pose;
    return kSuccess;
  }

  /**
   * @brief 把 `[x,y,yaw]` 序列追加到 map 坐标系 PoseArray。
   * @note 不清空已有 poses。
   */
  static ErrorType GetRosPoseArrayFromState3dVector(
      const std::vector<Vec3f>& states,
      geometry_msgs::msg::PoseArray* p_pose_array) {
    p_pose_array->header.frame_id = "map";
    p_pose_array->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    for (const auto& state : states) {
      geometry_msgs::msg::Pose pose;
      GetRosPoseFrom3DofState(state, &pose);
      p_pose_array->poses.push_back(pose);
    }
    return kSuccess;
  }

  /**
   * @brief 把平面状态序列的 x/y 追加为 z=0 的旧式 sensor_msgs/PointCloud。
   * @note yaw 被忽略，已有 points/channels 保留。
   */
  static ErrorType GetRosPointCloudFrom3DofStateVector(
      const std::vector<Vec3f>& states, sensor_msgs::msg::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    for (const auto& state : states) {
      Vec2f vec(state(0), state(1));
      geometry_msgs::msg::Point32 pt;
      ConvertVectorToPoint32<2>(vec, &pt);
      p_pc->points.push_back(pt);
    }
    return kSuccess;
  }

  /**
   * @brief 以固定 0.2 参数步长采样 CircleArc 并追加到 PointCloud。
   * @note CircleArc 的负弧长需要负步长，但此处固定正 0.2，可能继承无法结束的风险。
   */
  static ErrorType GetRosPointCloudFromCircleArc(
      const CircleArc& arc, sensor_msgs::msg::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    std::vector<Vec3f> states;
    // 下层状态转 PointCloud 会再次覆盖 header 时间戳。
    arc.GetSampledStates(0.2, &states);
    GetRosPointCloudFrom3DofStateVector(states, p_pc);
    return kSuccess;
  }

  /**
   * @brief 遍历共享起点的所有 CircleArc 候选并把采样点追加到同一 PointCloud。
   */
  static ErrorType GetRosPointCloudFromCircleArcBranch(
      const CircleArcBranch& arc_branch, sensor_msgs::msg::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    // circle_arc_vec() 返回副本；每条弧的转换还会重复更新时间戳。
    for (const auto& arc : arc_branch.circle_arc_vec()) {
      GetRosPointCloudFromCircleArc(arc, p_pc);
    }
    return kSuccess;
  }

  /**
   * @brief 将 common::Point 列表逐项追加为 Point32 点云。
   * @note Point 中附加 values 不参与可视化。
   */
  static ErrorType GetRosPointCloudFromPointList(
      const std::vector<common::Point>& point_list,
      sensor_msgs::msg::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    for (const auto& p : point_list) {
      geometry_msgs::msg::Point32 pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      p_pc->points.push_back(pt);
    }

    return kSuccess;
  }

  /**
   * @brief 由三维位置、颜色、尺度和 id 构造 SPHERE Marker。
   * @note 不设置 header/namespace，且没有把默认全零 orientation 改为单位四元数。
   */
  static ErrorType GetRosMarkerSphereUsingPoint(
      const Vec3f& pt, const ColorARGB& color, const Vec3f& scale,
      const int& id, visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::SPHERE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    FillColorInMarker(color, p_marker);
    FillScaleInMarker(scale, p_marker);
    p_marker->pose.position.x = pt(0);
    p_marker->pose.position.y = pt(1);
    p_marker->pose.position.z = pt(2);
    return kSuccess;
  }

  /**
   * @brief 将二维 Circle 构造成直径 `2*radius`、固定高度 1 的 CYLINDER Marker。
   * @note 不设置 header/namespace/有效 orientation，也不验证半径非负。
   */
  static ErrorType GetRosMarkerCylinderUsingCircle(
      const Circle& circle, const ColorARGB& color, const int& id,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::CYLINDER;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = circle.radius * 2;
    p_marker->scale.y = circle.radius * 2;
    p_marker->scale.z = 1;
    p_marker->pose.position.x = circle.center.x;
    p_marker->pose.position.y = circle.center.y;
    p_marker->pose.position.z = 0;
    return kSuccess;
  }

  /**
   * @brief 以 common::Point 为圆柱中心，使用调用方尺度/颜色/id 构造 CYLINDER。
   * @note 不设置 header/namespace/有效 orientation。
   */
  static ErrorType GetRosMarkerCylinderUsingPoint(
      const Point& pt, const Vec3f& scale, const ColorARGB& color,
      const int& id, visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::CYLINDER;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    FillColorInMarker(color, p_marker);
    FillScaleInMarker(scale, p_marker);
    p_marker->pose.position.x = pt.x;
    p_marker->pose.position.y = pt.y;
    p_marker->pose.position.z = pt.z;
    return kSuccess;
  }

  /**
   * @brief 从 ROS Pose 提取平面 `[x,y,yaw]`。
   * @note z、roll、pitch 被丢弃，输入四元数不做有限性或归一化检查。
   */
  static ErrorType Get3DofStateFromRosPose(const geometry_msgs::msg::Pose& pose,
                                           Vec3f* p_state) {
    (*p_state)(0) = pose.position.x;
    (*p_state)(1) = pose.position.y;
    // 先把消息四元数转换为 tf2，再分解 roll/pitch/yaw。
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (*p_state)(2) = yaw;
    return kSuccess;
  }

  /**
   * @brief 将二维 OBB 挤出为指定高度的有向 CUBE Marker。
   * @note OBB length 映射 scale.x，width 映射 scale.y，中心 z 固定为 0。
   */
  static ErrorType GetRosMarkerCubeUsingOrientedBoundingBox2D(
      const OrientedBoundingBox2D& obb, const ColorARGB& color,
      const decimal_t& scale_z, visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::CUBE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;

    Vec3f scale(obb.length, obb.width, scale_z);
    FillScaleColorInMarker(scale, color, p_marker);
    geometry_msgs::msg::Pose obb_pose;
    GetRosPoseFrom3DofState(Vec3f(obb.x, obb.y, obb.angle), &obb_pose);
    p_marker->pose = obb_pose;
    return kSuccess;
  }

  /**
   * @brief 构造带固定中心高度 offset_z 的二维 OBB CUBE Marker。
   */
  static ErrorType GetRosMarkerCubeUsingOrientedBoundingBox2DWithOffsetZ(
      const OrientedBoundingBox2D& obb, const decimal_t offset_z,
      const ColorARGB& color, const decimal_t& scale_z,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::CUBE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;

    Vec3f scale(obb.length, obb.width, scale_z);
    FillScaleColorInMarker(scale, color, p_marker);
    geometry_msgs::msg::Pose obb_pose;
    GetRosPoseFrom3DofState(Vec3f(obb.x, obb.y, obb.angle), &obb_pose);
    obb_pose.position.z = offset_z;
    p_marker->pose = obb_pose;
    return kSuccess;
  }

  /**
   * @brief 由中心坐标和三维总长度构造轴对齐 CUBE Marker。
   */
  static ErrorType GetRosMarkerCubeUsingAxisAlignedBoundingBox3D(
      const AxisAlignedBoundingBoxND<3>& aabb, const ColorARGB& color,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::CUBE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;

    Vec3f scale(aabb.len[0], aabb.len[1], aabb.len[2]);
    FillScaleColorInMarker(scale, color, p_marker);

    p_marker->pose.position.x = aabb.coord[0];
    p_marker->pose.position.y = aabb.coord[1];
    p_marker->pose.position.z = aabb.coord[2];
    return kSuccess;
  }

  /**
   * @brief 由整数上下界差和中点构造轴对齐 CUBE Marker。
   * @note 尺度使用 `upper-lower`，不为离散闭区间额外加一个单元宽度。
   */
  static ErrorType GetRosMarkerCubeUsingAxisAlignedCube3D(
      const AxisAlignedCubeNd<int, 3>& aabb, const ColorARGB& color,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::CUBE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;

    decimal_t x_len = aabb.upper_bound[0] - aabb.lower_bound[0];
    decimal_t y_len = aabb.upper_bound[1] - aabb.lower_bound[1];
    decimal_t z_len = aabb.upper_bound[2] - aabb.lower_bound[2];

    Vec3f scale(x_len, y_len, z_len);
    FillScaleColorInMarker(scale, color, p_marker);

    p_marker->pose.position.x =
        (aabb.upper_bound[0] + aabb.lower_bound[0]) / 2.0;
    p_marker->pose.position.y =
        (aabb.upper_bound[1] + aabb.lower_bound[1]) / 2.0;
    p_marker->pose.position.z =
        (aabb.upper_bound[2] + aabb.lower_bound[2]) / 2.0;
    return kSuccess;
  }

  /**
   * @brief 在 OBB 中心/航向处放置固定 BMW X5 网格资源。
   * @note 网格尺度固定为 1，不随 OBB length/width 缩放；使用嵌入材质并额外旋转模型轴。
   */
  static ErrorType GetRosMarkerMeshUsingOrientedBoundingBox2D(
      const OrientedBoundingBox2D& obb, const ColorARGB& color,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->mesh_resource = "package://common/materials/bmw_x5.dae";
    p_marker->mesh_use_embedded_materials = true;
    FillScaleInMarker(Vec3f(1.0, 1.0, 1.0), p_marker);
    FillColorInMarker(color, p_marker);
    geometry_msgs::msg::Pose obb_pose;
    GetRosPoseFrom3DofState(Vec3f(obb.x, obb.y, obb.angle), &obb_pose);
    p_marker->pose = obb_pose;
    // 固定四元数把 DAE 模型自身坐标轴对齐到车辆平面姿态。
    tf2::Quaternion q(0.0, -0.7071, -0.7071, 0.0);
    tf2::Quaternion obb_orientation(
        obb_pose.orientation.x, obb_pose.orientation.y,
        obb_pose.orientation.z, obb_pose.orientation.w);

    tf2::Quaternion result = obb_orientation * q;
    tf2::convert(result, p_marker->pose.orientation);
    return kSuccess;
  }

  /**
   * @brief 在 map 坐标系指定位置放置固定 traffic_cone.dae 网格。
   * @note 尺度固定为 2，姿态使用固定模型轴校正四元数。
   */
  static ErrorType GetRosMarkerMeshConeUsingPosition(
      const Vec3f& pos, const ColorARGB& color, const int& id,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    p_marker->id = id;
    p_marker->type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->mesh_resource = "package://common/materials/traffic_cone.dae";
    // 不启用嵌入材质，使用调用方颜色。
    FillScaleInMarker(Vec3f(2, 2, 2), p_marker);
    FillColorInMarker(color, p_marker);
    p_marker->pose.position.x = pos(0);
    p_marker->pose.position.y = pos(1);
    p_marker->pose.position.z = pos(2);
    tf2::Quaternion q(0.0, -0.7071, -0.7071, 0.0);
    tf2::convert(q, p_marker->pose.orientation);
    return kSuccess;
  }

  /**
   * @brief 由 `[x,y,yaw]` 和高度偏移构造固定 hexagon_sign.dae 网格 Marker。
   * @note 尺度固定为 0.5，使用调用方颜色和 map 当前时间。
   */
  static ErrorType GetRosMarkerMeshHexagonSignUsingPosition(
      const Vec3f& state, const decimal_t& z_offset, const ColorARGB& color,
      const int& id, visualization_msgs::msg::Marker* p_marker) {
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    p_marker->id = id;
    p_marker->type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->mesh_resource = "package://common/materials/hexagon_sign.dae";
    // 不启用嵌入材质。
    FillScaleInMarker(Vec3f(0.5, 0.5, 0.5), p_marker);
    FillColorInMarker(color, p_marker);
    geometry_msgs::msg::Pose pose;
    GetRosPoseFrom3DofState(state, &pose);
    pose.position.z = z_offset;
    p_marker->pose = pose;
    return kSuccess;
  }

  /**
   * @brief 用起点和向量终点向 ARROW Marker 追加两个几何点。
   * @note 不设置 id/header/颜色/尺度，也不清空已有 points。
   */
  static ErrorType GetRosMarkerArrowUsingOriginAndVector(
      const Vec3f& origin, const Vec3f& vec, visualization_msgs::msg::Marker* p_mk) {
    p_mk->type = visualization_msgs::msg::Marker::ARROW;
    p_mk->action = visualization_msgs::msg::Marker::MODIFY;

    geometry_msgs::msg::Point pt0, pt1;
    pt0.x = origin(0);
    pt0.y = origin(1);
    pt0.z = origin(2);
    pt1.x = origin(0) + vec(0);
    pt1.y = origin(1) + vec(1);
    pt1.z = origin(2) + vec(2);

    p_mk->points.push_back(pt0);
    p_mk->points.push_back(pt1);

    return kSuccess;
  }

  /**
   * @brief 使用 Pose 的局部 x 轴和给定长度构造姿态式 ARROW Marker。
   * @note 长度下限为 0.15；负 norm 也会显示为正向 0.15，而不是反向箭头。
   */
  static ErrorType GetRosMarkerArrowUsingPoseAndNorm(
      const geometry_msgs::msg::Pose& pose, const double& norm,
      const ColorARGB& color, visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::ARROW;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->pose = pose;
    // RViz 不接受零长度，因此设置固定最小长度。
    p_marker->scale.x = std::max(0.15, norm);
    p_marker->scale.y = 0.15;
    p_marker->scale.z = 0.15;
    FillColorInMarker(color, p_marker);
    return kSuccess;
  }

  /**
   * @brief 把二维点序列按 z=0 追加为单色 LINE_STRIP。
   * @note 只使用 scale.x 作为线宽，scale.y/z 被忽略。
   */
  static ErrorType GetRosMarkerLineStripUsing2DofVec(
      const vec_E<Vec2f>& path, const ColorARGB& color, const Vec3f& scale,
      const int& id, visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::msg::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = 0.0;
      p_marker->points.push_back(pt);
    }
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief 把二维点序列放置在固定 z 高度并追加为单色 LINE_STRIP。
   */
  static ErrorType GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
      const vec_E<Vec2f>& path, const ColorARGB& color, const Vec3f& scale,
      const decimal_t& z, const int& id, visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::msg::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = z;
      p_marker->points.push_back(pt);
    }
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief 从三维状态序列读取 x/y、忽略第三分量，并使用统一 z_offset 构造线条。
   */
  static ErrorType GetRosMarkerLineStripUsing3DofStateVec(
      const std::vector<Vec3f>& path, const decimal_t z_offset,
      const ColorARGB& color, const Vec3f& scale, const int& id,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::msg::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = z_offset;
      p_marker->points.push_back(pt);
    }
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief 在三维位置放置面向相机的文本 Marker。
   * @note 不设置 header/namespace，orientation 保持消息默认值。
   */
  static ErrorType GetRosMarkerTextUsingPositionAndString(
      const Vec3f& pos, const std::string& str, const ColorARGB& color,
      const Vec3f& scale, const int& id, visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    p_marker->pose.position.x = pos(0);
    p_marker->pose.position.y = pos(1);
    p_marker->pose.position.z = pos(2);
    p_marker->text = str;
    FillColorInMarker(color, p_marker);
    FillScaleInMarker(scale, p_marker);
    return kSuccess;
  }

  /**
   * @brief 构造固定高度的 LINE_STRIP，并按点顺序追加 Jet 渐变 colors。
   * @note 调用 `FillGradientColorInMarker(false,...)`，但该参数当前被忽略。
   */
  static ErrorType GetRosMarkerLineStripGradientColorUsing3DofStateVec(
      const std::vector<Vec3f>& path, const Vec3f& scale,
      const decimal_t& offset_z, const int& id,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::msg::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = offset_z;
      p_marker->points.push_back(pt);
    }
    FillGradientColorInMarker(0, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief 把 common::Point 数组复制并追加为带完整三维坐标的 LINE_STRIP。
   * @note points 参数按值传递，会复制整个输入数组。
   */
  static ErrorType GetRosMarkerLineStripUsingPoints(
      const std::vector<Point> points, const Vec3f& scale,
      const ColorARGB& color, const int& id,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = id;
    FillScaleColorInMarker(scale, color, p_marker);
    for (const auto& p : points) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      p_marker->points.push_back(point);
    }
    return kSuccess;
  }

  /**
   * @brief 为单车构造车身/网格、速度箭头和速度文本，并追加到 MarkerArray。
   * @param id 当前 Marker 的基础 id；内部预留 id 到 id+6。
   * @note 车辆 id==0 时显示网格，其他车辆显示 OBB；转向辅助 Marker 被计算但不发布。
   */
  static ErrorType GetRosMarkerArrayUsingVehicle(
      const Vehicle& vehicle, const ColorARGB& color_obb,
      const ColorARGB& color_vel_vec, const ColorARGB& color_steer,
      const int& id, visualization_msgs::msg::MarkerArray* p_marker_array) {
    // 本次构造的所有 Marker 共用一个 ROS 时间戳。
    rclcpp::Time ros_time = rclcpp::Clock(RCL_ROS_TIME).now();

    // 车身 OBB 以固定 1.7 m 高度挤出，中心抬到 z=0.75。
    visualization_msgs::msg::Marker obb_marker;
    obb_marker.header.frame_id = "map";
    obb_marker.header.stamp = ros_time;
    obb_marker.id = id;
    OrientedBoundingBox2D obb = vehicle.RetOrientedBoundingBox();
    GetRosMarkerCubeUsingOrientedBoundingBox2D(obb, color_obb, 1.7,
                                               &obb_marker);
    obb_marker.pose.position.z = 0.75;

    // 车辆网格使用固定 BMW X5 资源，不按 VehicleParam 尺寸缩放。
    visualization_msgs::msg::Marker mesh_marker;
    mesh_marker.header.frame_id = "map";
    mesh_marker.header.stamp = ros_time;
    mesh_marker.id = id + 1;
    GetRosMarkerMeshUsingOrientedBoundingBox2D(obb, color_obb, &mesh_marker);

    // 速度箭头沿车辆 yaw 的局部 x 轴，负速度会被最小正长度规则显示为前向短箭头。
    visualization_msgs::msg::Marker vel_vec_marker;
    vel_vec_marker.header.frame_id = "map";
    vel_vec_marker.header.stamp = ros_time;
    vel_vec_marker.id = id + 2;
    // vel_vec_marker.ns = std::string("vel_vec");
    geometry_msgs::msg::Pose pose;
    GetRosPoseFrom3DofState(vehicle.Ret3DofState(), &pose);
    pose.position.z = 0.8;
    GetRosMarkerArrowUsingPoseAndNorm(pose, vehicle.state().velocity,
                                      color_vel_vec, &vel_vec_marker);

    // 文本显示 `车辆ID_速度 m/s`，通过复用 Vec3f 第三分量指定 z=3 m。
    visualization_msgs::msg::Marker vel_text_marker;
    vel_text_marker.header.frame_id = "map";
    vel_text_marker.header.stamp = ros_time;
    auto pos = vehicle.Ret3DofState();
    pos(2) = 3.0;
    std::string str;
    str += std::string(std::to_string(vehicle.id()) + "_");
    decimal_t visualized_vel = vehicle.state().velocity;
    str += std::string(
        GetStringByValueWithPrecision<decimal_t>(visualized_vel, 3) + " m/s\n");
    GetRosMarkerTextUsingPositionAndString(pos, str, cmap.at("black"),
                                           Vec3f(0.75, 0.75, 0.75), id + 3,
                                           &vel_text_marker);

    // 以下三类转向辅助图元会完整计算，但当前 push_back 语句被注释，不会进入输出数组。
    double arc_length = 10;
    visualization_msgs::msg::Marker steering_angle_marker;
    steering_angle_marker.header.frame_id = "map";
    steering_angle_marker.header.stamp = ros_time;
    Vec3f state(vehicle.state().vec_position(0),
                vehicle.state().vec_position(1), vehicle.state().angle);

    common::CircleArc arc(state, vehicle.state().curvature, arc_length);
    std::vector<Vec3f> arc_samples;
    arc.GetSampledStates(0.2, &arc_samples);
    GetRosMarkerLineStripUsing3DofStateVec(arc_samples, +0.2, color_steer,
                                           Vec3f(0.1, 0, 0), id + 4,
                                           &steering_angle_marker);

    // 反向延长圆弧通过航向加 pi、曲率取反构造。
    visualization_msgs::msg::Marker steering_angle_marker_reverse;
    steering_angle_marker_reverse.header.frame_id = "map";
    steering_angle_marker_reverse.header.stamp = ros_time;
    Vec3f state_reverse = state;
    state_reverse(2) = normalize_angle(kPi + state_reverse(2));
    common::CircleArc arc2(state_reverse, -1.0 * vehicle.state().curvature,
                           arc_length);
    std::vector<Vec3f> arc_samples2;
    arc2.GetSampledStates(0.2, &arc_samples2);
    GetRosMarkerLineStripUsing3DofStateVec(arc_samples2, +0.2, color_steer,
                                           Vec3f(0.1, 0, 0), id + 5,
                                           &steering_angle_marker_reverse);

    // 横向短线表示车辆局部横轴，但其状态 z 会被下层统一 z_offset 覆盖。
    visualization_msgs::msg::Marker horizontal_marker;
    horizontal_marker.header.frame_id = "map";
    horizontal_marker.header.stamp = ros_time;
    auto state3df = vehicle.Ret3DofState();
    std::vector<Vecf<3>> horizontal_line;
    const decimal_t line_width = 1.7;
    horizontal_line.emplace_back(state3df[0] + line_width * sin(state3df[2]),
                                 state3df[1] - line_width * cos(state3df[2]),
                                 -0.2);
    horizontal_line.emplace_back(state3df[0] - line_width * sin(state3df[2]),
                                 state3df[1] + line_width * cos(state3df[2]),
                                 -0.2);
    GetRosMarkerLineStripUsing3DofStateVec(horizontal_line, -0.4, color_steer,
                                           Vec3f(0.1, 0, 0), id + 6,
                                           &horizontal_marker);

    // 约定 id==0 为自车，显示网格；周车使用简单 OBB。
    if (vehicle.id() == 0) {
      p_marker_array->markers.push_back(mesh_marker);
    } else {
      p_marker_array->markers.push_back(obb_marker);
    }
    p_marker_array->markers.push_back(vel_vec_marker);
    p_marker_array->markers.push_back(vel_text_marker);
    // 转向圆弧和横轴目前不发布，但上述采样开销仍然发生。
    // p_marker_array->markers.push_back(steering_angle_marker);
    // p_marker_array->markers.push_back(steering_angle_marker_reverse);
    // p_marker_array->markers.push_back(horizontal_marker);

    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Using Circle Obstacle object
   *
   * @param obs
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerUsingCircleObstacle(
      const CircleObstacle& obs, visualization_msgs::msg::Marker* p_marker) {
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    GetRosMarkerCylinderUsingCircle(obs.circle, ColorARGB(0.5, 1.0, 1.0, 1.0),
                                    obs.id, p_marker);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Using Polygon Obstacle object
   *
   * @param obs
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerUsingPolygonObstacle(
      const PolygonObstacle& obs, const int& id,
      visualization_msgs::msg::Marker* p_marker) {
    std::vector<Point> points = obs.polygon.points;
    points.push_back(*(points.begin()));
    for (auto& p : points) {
      p.z = -0.2;
    }
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    GetRosMarkerLineStripUsingPoints(
        points, Vec3f(0.2, 0, 0), ColorARGB(1.0, 0.7, 0.7, 0.7), id, p_marker);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Using Obstacle Set object
   *
   * @param obstacles
   * @param p_marker_array
   * @return ErrorType
   */
  static ErrorType GetRosMarkerUsingObstacleSet(
      const ObstacleSet& obstacles,
      visualization_msgs::msg::MarkerArray* p_marker_array) {
    for (const auto& p_obs : obstacles.obs_circle) {
      visualization_msgs::msg::Marker obs_marker;
      GetRosMarkerUsingCircleObstacle(p_obs.second, &obs_marker);
      p_marker_array->markers.push_back(obs_marker);
    }
    int id_cnt = 0;
    for (const auto& p_obs : obstacles.obs_polygon) {
      switch (p_obs.second.type) {
        case 0: {
          visualization_msgs::msg::Marker obs_marker;
          GetRosMarkerUsingPolygonObstacle(p_obs.second, id_cnt, &obs_marker);
          p_marker_array->markers.push_back(obs_marker);
          ++id_cnt;
          break;
        }
        case 1: {
          for (const auto& pt : p_obs.second.polygon.points) {
            visualization_msgs::msg::Marker obs_marker;
            Vec3f pos(pt.x, pt.y, 0.4);
            GetRosMarkerMeshConeUsingPosition(pos, cmap.at("yellow"), id_cnt,
                                              &obs_marker);
            p_marker_array->markers.push_back(obs_marker);
            ++id_cnt;
          }
          break;
        }
        default:
          break;
      }
    }
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Arr Using Semantic Behavior object
   *
   * @param behavior
   * @param p_marker_array
   * @return ErrorType
   */
  static ErrorType GetRosMarkerArrUsingSemanticBehavior(
      const SemanticBehavior& behavior,
      visualization_msgs::msg::MarkerArray* p_marker_array) {
    // lane direction marker
    {
      if (behavior.ref_lane.IsValid()) {
        visualization_msgs::msg::Marker direction_mk;
        direction_mk.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        direction_mk.header.frame_id = std::string("map");
        direction_mk.id = 0;
        direction_mk.type = visualization_msgs::msg::Marker::LINE_LIST;
        direction_mk.action = visualization_msgs::msg::Marker::MODIFY;
        decimal_t angle = 0.0;  // angle between horizontal line & direction
        if (behavior.lat_behavior == common::LateralBehavior::kLaneChangeLeft ||
            behavior.lat_behavior ==
                common::LateralBehavior::kLaneChangeRight) {
          FillScaleColorInMarker(Vec3f(0.4, 0.0, 0.0),
                                 ColorARGB(1.0, 1.0, 1.0, 0.0), &direction_mk);
          angle = kPi / 4.0;
        } else if (behavior.lon_behavior ==
                   common::LongitudinalBehavior::kDecelerate) {
          FillScaleColorInMarker(Vec3f(0.4, 0.0, 0.0),
                                 ColorARGB(1.0, 1.0, 0.0, 0.0), &direction_mk);
          angle = 0.0;
        } else {
          FillScaleColorInMarker(Vec3f(0.4, 0.0, 0.0),
                                 ColorARGB(1.0, 0.1, 0.8, 0.1), &direction_mk);
          angle = kPi / 4.0;
        }

        const decimal_t sample_step = 5.0;
        const decimal_t arrow_width = 0.75;
        for (decimal_t s = behavior.ref_lane.begin();
             s < behavior.ref_lane.end(); s += sample_step) {
          Vecf<2> pos;
          behavior.ref_lane.GetPositionByArcLength(s, &pos);
          Vecf<2> normal_vec;
          behavior.ref_lane.GetNormalVectorByArcLength(s, &normal_vec);

          geometry_msgs::msg::Point origin;
          ConvertVectorToPoint<2>(pos, &origin);
          {
            geometry_msgs::msg::Point left_arrow;
            Vecf<2> left = pos + arrow_width / acos(angle) *
                                     rotate_vector_2d(normal_vec, angle);
            ConvertVectorToPoint<2>(left, &left_arrow);
            direction_mk.points.push_back(origin);
            direction_mk.points.push_back(left_arrow);
          }
          {
            geometry_msgs::msg::Point right_arrow;
            Vecf<2> right = pos + arrow_width / acos(angle) *
                                      rotate_vector_2d(-normal_vec, -angle);
            ConvertVectorToPoint<2>(right, &right_arrow);
            direction_mk.points.push_back(origin);
            direction_mk.points.push_back(right_arrow);
          }
        }
        p_marker_array->markers.push_back(direction_mk);
      }  // end if valid
    }

    // visualize curvature
    {
      decimal_t sample_step = 1.0;
      if (behavior.ref_lane.IsValid()) {
        visualization_msgs::msg::Marker curvature_mk;
        curvature_mk.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        curvature_mk.header.frame_id = std::string("map");
        curvature_mk.id = 1;
        curvature_mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
        curvature_mk.action = visualization_msgs::msg::Marker::MODIFY;
        curvature_mk.scale.x = 0.2;
        for (decimal_t s = behavior.ref_lane.begin();
             s < behavior.ref_lane.end(); s += sample_step) {
          Vecf<2> pos;
          behavior.ref_lane.GetPositionByArcLength(s, &pos);
          geometry_msgs::msg::Point origin;
          ConvertVectorToPoint<2>(pos, &origin);
          curvature_mk.points.push_back(origin);

          decimal_t c, cc;
          behavior.ref_lane.GetCurvatureByArcLength(s, &c, &cc);
          common::ColorARGB color =
              common::GetJetColorByValue(fabs(c), 0.4, 0.0);
          std_msgs::msg::ColorRGBA c_ros;
          c_ros.a = color.a;
          c_ros.r = color.r;
          c_ros.g = color.g;
          c_ros.b = color.b;
          curvature_mk.colors.push_back(c_ros);
        }
        p_marker_array->markers.push_back(curvature_mk);
      }
    }

    // visualize longitudinal behavior
    {
      if (behavior.ref_lane.IsValid()) {
        common::ColorARGB clr(1.0, 1.0, 0, 0);
        decimal_t length = 0.0;
        switch (behavior.lon_behavior) {
          case common::LongitudinalBehavior::kMaintain: {
            length = 0.0;
            clr = common::ColorARGB(1.0, 0.0, 1.0, 0.0);
            break;
          }
          case common::LongitudinalBehavior::kAccelerate: {
            length = 0.75;
            clr = common::ColorARGB(1.0, 1.0, 1.0, 0);
            break;
          }
          case common::LongitudinalBehavior::kDecelerate: {
            length = -0.75;
            clr = common::ColorARGB(1.0, 1.0, 0, 0);
            break;
          }
          default:
            break;
        }

        visualization_msgs::msg::Marker lon_mk;
        lon_mk.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        lon_mk.header.frame_id = std::string("map");
        lon_mk.id = 3;
        lon_mk.type = visualization_msgs::msg::Marker::ARROW;
        lon_mk.action = visualization_msgs::msg::Marker::MODIFY;
        geometry_msgs::msg::Point pt0, pt1;
        pt0.x = behavior.state.vec_position(0);
        pt0.y = behavior.state.vec_position(1);
        pt0.z = 2.5;
        pt1.x = behavior.state.vec_position(0);
        pt1.y = behavior.state.vec_position(1);
        pt1.z = pt0.z + length;
        lon_mk.points.push_back(pt0);
        lon_mk.points.push_back(pt1);
        lon_mk.scale.x = 0.2;
        lon_mk.scale.y = 0.4;
        FillColorInMarker(clr, &lon_mk);
        p_marker_array->markers.push_back(lon_mk);
      }
    }

    // forward trajs
    {
      auto surround_trajs_set = behavior.surround_trajs;
      common::ColorARGB traj_color = cmap.at("sky blue");
      traj_color.a = 0.8;
      double traj_z = 0.3;
      int cnt = 100;

      for (const auto& surround_trajs : surround_trajs_set) {
        for (const auto& p_traj : surround_trajs) {
          std::vector<common::Point> points;
          for (const auto& v : p_traj.second) {
            common::Point pt(v.state().vec_position(0),
                             v.state().vec_position(1));
            pt.z = traj_z;
            points.push_back(pt);
            visualization_msgs::msg::Marker point_marker;

            point_marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            point_marker.header.frame_id = std::string("map");

            common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
                common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, ++cnt,
                &point_marker);
            p_marker_array->markers.push_back(point_marker);
          }
          visualization_msgs::msg::Marker line_marker;
          line_marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
          line_marker.header.frame_id = std::string("map");
          common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
              points, Vec3f(0.1, 0.1, 0.1), traj_color, ++cnt, &line_marker);
          p_marker_array->markers.push_back(line_marker);
        }
      }
    }

    return kSuccess;
  }

  /**
   * @brief Convert GridMapND<T, 2> to nav_msgs::msg::OccupancyGrid
   *
   * @tparam T Data type
   * @param GridMapND in type T
   * @param time_stamp ROS timestamp
   * @param p_occ_grid Pointer of ROS nav_msgs::msg::OccupancyGrid
   * @return ErrorType
   */
  template <typename T>
  static ErrorType GetRosOccupancyGridUsingGripMap2D(
      const GridMapND<T, 2>& grid_map, const rclcpp::Time& time_stamp,
      nav_msgs::msg::OccupancyGrid* p_occ_grid) {
    p_occ_grid->header.frame_id = "map";
    p_occ_grid->header.stamp = time_stamp;
    p_occ_grid->info.height = grid_map.dims_size(0);
    p_occ_grid->info.width = grid_map.dims_size(1);
    p_occ_grid->info.resolution = grid_map.dims_resolution(0);
    geometry_msgs::msg::Pose origin;
    Vec3f origin_pose(grid_map.origin()[0], grid_map.origin()[1], 0.0);
    GetRosPoseFrom3DofState(origin_pose, &origin);
    p_occ_grid->info.origin = origin;
    p_occ_grid->info.map_load_time = time_stamp;

    p_occ_grid->data.resize(grid_map.data_size());
    std::copy(grid_map.data()->begin(), grid_map.data()->end(),
              p_occ_grid->data.begin());
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cube List Using Grip Map 3 D object
   *
   * @tparam T
   * @param p_grid_map
   * @param time_stamp
   * @param frame_id
   * @param pose
   * @param p_marker
   * @return ErrorType
   */
  template <typename T>
  static ErrorType GetRosMarkerCubeListUsingGripMap3D(
      const GridMapND<T, 3>* p_grid_map, const rclcpp::Time& time_stamp,
      const std::string& frame_id, const Vec3f& pose,
      visualization_msgs::msg::Marker* p_marker) {
    p_marker->header.frame_id = frame_id;
    p_marker->header.stamp = time_stamp;
    p_marker->type = visualization_msgs::msg::Marker::CUBE_LIST;
    p_marker->action = visualization_msgs::msg::Marker::MODIFY;
    p_marker->id = 0;

    geometry_msgs::msg::Pose pose_origin;
    GetRosPoseFrom3DofState(pose, &pose_origin);
    p_marker->pose = pose_origin;

    p_marker->scale.x = p_grid_map->dims_resolution(0);
    p_marker->scale.y = p_grid_map->dims_resolution(1);
    p_marker->scale.z = p_grid_map->dims_resolution(2);

    auto origin = p_grid_map->origin();

    int ele_num = p_grid_map->data_size();
    p_marker->points.reserve(ele_num);
    p_marker->colors.reserve(ele_num);

    const T* map_ptr = p_grid_map->data_ptr();
    // int z_max = p_grid_map->dims_size(2);
    std::array<int, 3> idx;
    std::array<decimal_t, 3> p_w;
    for (int i = 0; i < p_grid_map->data_size(); ++i) {
      if (i > p_grid_map->dims_step(2)) {
        break;
      }

      if (*(map_ptr + i) == 0) continue;

      idx = p_grid_map->GetNDimIdxUsingMonoIdx(i);
      p_grid_map->GetGlobalPositionUsingCoordinate(idx, &p_w);

      geometry_msgs::msg::Point pt;
      pt.x = p_w[0];
      pt.y = p_w[1];
      pt.z = p_w[2] - origin[2];
      p_marker->points.push_back(pt);

      std_msgs::msg::ColorRGBA clr_ros;
      // ColorARGB clr = GetJetColorByValue(idx[2], z_max, 0);
      // clr_ros.a = 0.95;
      // clr_ros.r = clr.r;
      // clr_ros.g = clr.g;
      // clr_ros.b = clr.b;

      clr_ros.a = 0.5;
      clr_ros.r = 1.0;
      clr_ros.g = 0.0;
      clr_ros.b = 0.0;
      p_marker->colors.push_back(clr_ros);
    }
    return kSuccess;
  }
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__
