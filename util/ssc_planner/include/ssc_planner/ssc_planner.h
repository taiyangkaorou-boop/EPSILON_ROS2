/**
 * @file ssc_planner.h
 * @author HKUST Aerial Robotics Group
 * @brief 基于时空语义走廊和 Bezier QP 的 Frenet 轨迹规划器。
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_

#include <memory>
#include <set>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/primitive/frenet_primitive.h"
#include "common/spline/spline_generator.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_bezier_traj.h"
#include "common/trajectory/frenet_primitive_traj.h"
#include "ssc_config.pb.h"
#include "ssc_planner/map_interface.h"
#include "ssc_planner/ssc_map.h"

namespace planning {

/**
 * @brief 将行为层 rollout 转换为无碰撞 SSC，并生成连续 Frenet 轨迹。
 *
 * 每轮规划读取自车、参考 Lane、障碍物和多行为前向仿真，将所有状态/车身顶点批量投影到
 * Frenet 坐标。各候选行为分别构建时空走廊；正常速度使用五阶二维 Bezier spline 优化，
 * 低速使用 FrenetPrimitive 连接初末状态，最后按行为层选择结果或回退到车道保持。
 */
class SscPlanner : public Planner {
 public:
  // 地图、语义和轨迹类型别名。
  using ObstacleMapType = uint8_t;
  using SscMapDataType = uint8_t;

  using Lane = common::Lane;
  using State = common::State;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using FrenetState = common::FrenetState;
  using FrenetTrajectory = common::FrenetTrajectory;
  using FrenetPrimitive = common::FrenetPrimitive;
  using FrenetBezierTrajectory = common::FrenetBezierTrajectory;
  using FrenetPrimitiveTrajectory = common::FrenetPrimitiveTrajectory;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  typedef common::BezierSpline<5, 2> BezierSpline;

  /// 默认构造只初始化带类内默认值的成员，不分配 SscMap 或绑定地图接口。
  SscPlanner() = default;

  /// 绑定由外部服务端持有的地图接口；规划器不取得其所有权。
  ErrorType set_map_interface(SscPlannerMapItf* map_itf);

  /// 设置下一轮规划的一次性起始状态；未设置时使用地图快照自车状态。
  ErrorType set_initial_state(const State& state);

  /// 返回内部 SscMap 的可修改裸指针。
  SscMap* p_ssc_map() const { return p_ssc_map_; }

  /// 按值返回本轮规划起始状态的 Frenet 表示。
  FrenetState ego_frenet_state() const { return ego_frenet_state_; }

  /// 按值返回地图快照中的自车。
  Vehicle ego_vehicle() const { return ego_vehicle_; }

  /// 按值返回行为层提供的全部自车全局坐标 rollout。
  vec_E<vec_E<Vehicle>> forward_trajs() const { return forward_trajs_; }

  /// 按值返回所有成功候选的 Bezier spline；低速候选中可能保存默认 spline。
  vec_E<BezierSpline> qp_trajs() const { return qp_trajs_; }

  /// 返回本轮轨迹绝对时间原点。
  decimal_t time_origin() const { return time_origin_; }

  /// 返回预留的单组周车 Frenet 轨迹 map；当前实现没有填充该成员。
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs()
      const {
    return sur_vehicle_trajs_fs_;
  }

  /// 按候选行为返回周车 Frenet rollout。
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
  surround_forward_trajs_fs() const {
    return surround_forward_trajs_fs_;
  };

  /// 按候选行为返回自车 Frenet rollout。
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs() const {
    return forward_trajs_fs_;
  }

  /// 返回起始时刻自车车身在 Frenet 坐标中的顶点。
  vec_E<Vec2f> ego_vehicle_contour_fs() const {
    return fs_ego_vehicle_.vertices;
  }

  /// 返回带 Frenet 状态和车身顶点的起始自车。
  common::FsVehicle fs_ego_vehicle() const { return fs_ego_vehicle_; }

  // 旧的单 spline getter 已停用，候选 spline 统一由 qp_trajs() 返回。
  // BezierSpline bezier_spline() const { return bezier_spline_; }

  /**
   * @brief 以基类指针返回当前选定轨迹的独立堆对象。
   *
   * 低速模式复制 FrenetPrimitiveTrajectory，正常速度模式复制 FrenetBezierTrajectory。
   */
  std::unique_ptr<FrenetTrajectory> trajectory() const {
    if (!is_lateral_independent_) {
      return std::unique_ptr<FrenetPrimitiveTrajectory>(
          new FrenetPrimitiveTrajectory(low_spd_alternative_traj_));
    }
    return std::unique_ptr<FrenetBezierTrajectory>(
        new FrenetBezierTrajectory(trajectory_));
  }

  /// 按值返回本轮参考 Lane 对应的 StateTransformer。
  common::StateTransformer state_transformer() const { return stf_; }

  /// 返回最近一次 RunOnce 的墙钟耗时，单位 ms。
  decimal_t time_cost() const { return time_cost_; }

  /// 按值返回用于本轮规划的起始 Frenet 状态。
  common::FrenetState initial_frenet_state() const {
    return initial_frenet_state_;
  }

  /// 返回 Planner 接口使用的固定模块名。
  std::string Name() override;

  /// 读取 protobuf 文本配置、映射 SscMap 参数并分配内部 SscMap。
  ErrorType Init(const std::string config_path) override;

  /// 执行一次数据准备、Frenet 转换、走廊构建、优化、选择和计时。
  ErrorType RunOnce() override;

 private:
  /// 从文本文件解析 planning::ssc::Config。
  ErrorType ReadConfig(const std::string config_path);

  /// 检查连续 corridor 非空且相邻 cube 的时间边界严格相接。
  ErrorType CorridorFeasibilityCheck(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes);

  /// 打包全部自车/周车状态、车身顶点和障碍点，批量投影后按原结构拆包。
  ErrorType StateTransformForInputData();

  /// 逐候选建立起终约束和参考点，生成 Bezier spline 或低速 primitive。
  ErrorType RunQpOptimization();

  /// 采样检查轨迹初始位置/速度和全程曲率；RunOnce 当前未启用该验证。
  ErrorType ValidateTrajectory(const FrenetTrajectory& traj);

  /// 使用固定四线程 OpenMP 并行转换状态和点。
  ErrorType StateTransformUsingOpenMp(const vec_E<State>& global_state_vec,
                                      const vec_E<Vec2f>& global_point_vec,
                                      vec_E<FrenetState>* frenet_state_vec,
                                       vec_E<Vec2f>* fs_point_vec) const;

  /// 在当前线程依次转换状态和点。
  ErrorType StateTransformSingleThread(const vec_E<State>& global_state_vec,
                                       const vec_E<Vec2f>& global_point_vec,
                                       vec_E<FrenetState>* frenet_state_vec,
                                       vec_E<Vec2f>* fs_point_vec) const;

  /// 按行为层指定横向行为选择候选；无精确匹配时只回退 LaneKeeping。
  ErrorType UpdateTrajectoryWithCurrentBehavior();

  // 当前规划快照的自车、行为、Frenet 状态和参考 Lane。
  Vehicle ego_vehicle_;
  LateralBehavior ego_behavior_;
  FrenetState ego_frenet_state_;
  Lane nav_lane_local_;
  decimal_t time_origin_{0.0};

  // 下一轮一次性起始状态及其 Frenet 表示。
  State initial_state_;
  bool has_initial_state_ = false;

  common::FrenetState initial_frenet_state_;

  // 地图障碍物和行为层多候选全局坐标 rollout。
  GridMap2D grid_map_;
  std::set<std::array<decimal_t, 2>> obstacle_grids_;
  vec_E<vec_E<Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;

  // 由世界坐标障碍栅格中心转换得到的 Frenet 点。
  vec_E<Vec2f> obstacle_grids_fs_;

  // 优化初值：起始自车以及多候选自车/周车 Frenet rollout。
  common::FsVehicle fs_ego_vehicle_;
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs_;
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs_;
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
      surround_forward_trajs_fs_;

  // 通过可行性检查的候选轨迹、行为、走廊和参考状态，所有容器按下标对齐。
  vec_E<BezierSpline> qp_trajs_;
  vec_E<FrenetPrimitive> primitive_trajs_;
  std::vector<LateralBehavior> valid_behaviors_;
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> corridors_;
  vec_E<vec_E<common::FrenetState>> ref_states_list_;

  // 最终模式、轨迹和对应 corridor/reference；低速返回 primitive，高速返回 Bezier。
  bool is_lateral_independent_ = true;
  FrenetBezierTrajectory trajectory_;
  FrenetPrimitiveTrajectory low_spd_alternative_traj_;
  vec_E<common::SpatioTemporalSemanticCubeNd<2>> final_corridor_;
  vec_E<common::FrenetState> final_ref_states_;

  // 本轮参考 Lane 的笛卡尔/Frenet 坐标变换器。
  common::StateTransformer stf_;

  // 外部地图接口为借用指针，内部 SscMap 由 Init 用 new 分配。
  SscPlannerMapItf* map_itf_;
  bool map_valid_ = false;
  SscMap* p_ssc_map_;

  // 地图快照时间戳和本轮墙钟耗时。
  decimal_t stamp_ = 0.0;
  decimal_t time_cost_ = 0.0;

  /// protobuf 文本配置解析结果。
  planning::ssc::Config cfg_;
};

}  // namespace planning

#endif
