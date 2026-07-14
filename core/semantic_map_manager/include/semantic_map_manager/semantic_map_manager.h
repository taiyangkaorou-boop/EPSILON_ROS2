#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>

#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/math/calculations.h"
#include "common/mobil/mobil_behavior_prediction.h"
#include "common/rss/rss_checker.h"
#include "motion_predictor/onlane_fs_predictor.h"
#include "semantic_map_manager/config_loader.h"
#include "semantic_map_manager/traffic_signal_manager.h"
#include "vehicle_model/idm_model.h"

namespace semantic_map_manager {

/**
 * @brief 单 ego 的语义地图快照、Lane 拓扑、周车语义预测和规划查询服务。
 *
 * UpdateSemanticMap 接收 DataRenderer 派生数据，按顺序更新语义 Lane、本地 Lane/LUT、
 * 语义车辆、关键车辆和可选开环轨迹。类同时向行为/运动规划器提供碰撞、Lane、前后车、
 * 交通信号和拓扑距离查询；大部分状态以值对象保存在内部。
 */
class SemanticMapManager {
 public:
  using ObstacleMapType = uint8_t;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;
  using State = common::State;
  using Lane = common::Lane;
  using LateralBehavior = common::LateralBehavior;
  using SemanticLane = common::SemanticLane;

  /// 默认构造空快照；ego_id_ 和 p_config_loader_ 等成员不会被显式初始化。
  SemanticMapManager() {}

  /// 按 ego ID 和 JSON 路径创建 ConfigLoader、解析 AgentConfigInfo 并启动全局计时器。
  SemanticMapManager(const int &id, const std::string &agent_config_path);

  /// 直接设置搜索半径、开环预测和坐标轴约定，启用快速 LUT/简单 Lane 结构模式。
  SemanticMapManager(const int &id, const decimal_t surrounding_search_radius,
                     bool enable_openloop_prediction, bool use_right_hand_axis);

  /// 当前为空析构；配置构造器动态创建的 ConfigLoader 不会被释放。
  ~SemanticMapManager() {}

  /// 查询世界点对应 GridMap 单元是否严格等于 OCCUPIED。
  ErrorType CheckCollisionUsingGlobalPosition(const Vec2f &p_w,
                                               bool *res) const;

  /// 返回世界点对应的原始障碍物 GridMap 数值。
  ErrorType GetObstacleMapValueUsingGlobalPosition(const Vec2f &p_w,
                                                   ObstacleMapType *res);

  /// 检查给定车辆状态与静态栅格顶点、以及可选周车开环预测的碰撞。
  ErrorType CheckCollisionUsingStateAndVehicleParam(
      const common::VehicleParam &vehicle_param, const common::State &state,
      bool *res);

  /// 用两个车辆状态构造 OBB 并执行分离轴相交检查。
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b, bool *res);

  /// 预留状态序列碰撞检查；当前只有声明，没有实现定义。
  ErrorType CheckCollisionUsingStateVec(
      const vec_E<common::State> state_vec) const;

  /// 对全部语义 Lane 计算状态投影距离、弧长、航向差和 Lane ID，保留 10 m 内结果。
  ErrorType GetDistanceToLanesUsing3DofState(
      const Vec3f &state,
      std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const;

  /// 覆盖原始快照并依次重建 Lane、车辆语义、关键车辆、预测和可选日志。
  ErrorType UpdateSemanticMap(
      const double &time_stamp, const common::Vehicle &ego_vehicle,
      const common::LaneNet &whole_lane_net,
      const common::LaneNet &surrounding_lane_net,
      const common::GridMapND<ObstacleMapType, 2> &obstacle_map,
      const std::set<std::array<decimal_t, 2>> &obstacle_grids,
      const common::VehicleSet &surrounding_vehicles);

  /// 在附近 Lane 中优先按航向差选择最近 Lane，并输出距离和投影弧长。
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) const;

  /// 用横向偏移/速度固定阈值和相邻 Lane 可用性输出 one-hot LK/LCL/LCR 分布。
  ErrorType NaiveRuleBasedLateralBehaviorPrediction(
      const common::Vehicle &vehicle, const int nearest_lane_id,
      common::ProbDistOfLatBehaviors *lat_probs);

  /// 为 LK/LCL/LCR 构造参考 Lane 及前后车上下文，再调用通用 MOBIL 概率预测器。
  ErrorType MobilRuleBasedBehaviorPrediction(
      const common::Vehicle &vehicle, const common::VehicleSet &nearby_vehicles,
      common::ProbDistOfLatBehaviors *res);

  /// 沿给定 Lane 调用 OnLaneFsPredictor 生成指定时域/步长的车辆状态轨迹。
  ErrorType TrajectoryPredictionForVehicle(const common::Vehicle &vehicle,
                                           const common::Lane &lane,
                                           const decimal_t &t_pred,
                                            const decimal_t &t_step,
                                            vec_E<common::State> *traj);

  /// 从给定 Lane 沿 child 和可换相邻 Lane 做最多 20 节点 BFS，判断能否到达 path 任一 ID。
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) const;

  /// 先匹配当前/目标 Lane，再从 fast LUT 或动态前后拓展样本构造行为参考 Lane。
  ErrorType GetRefLaneForStateByBehavior(const common::State &state,
                                         const std::vector<int> &navi_path,
                                         const LateralBehavior &behavior,
                                         const decimal_t &max_forward_len,
                                         const decimal_t &max_back_len,
                                         const bool is_high_quality,
                                         common::Lane *lane) const;

  /// 把 LK/Undefined 映射当前 Lane，把可用 LCL/LCR 映射到对应相邻 Lane ID。
  ErrorType GetTargetLaneId(const int lane_id, const LateralBehavior &behavior,
                            int *target_lane_id) const;

  /// 沿目标 Lane 的 father/child 拓扑前后扩展，拼接原始点并以 1 m 步长输出局部样本。
  ErrorType GetLocalLaneSamplesByState(const common::State &state,
                                       const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) const;
  ErrorType GetLeadingVehicleOnLane(const common::Lane &ref_lane,
                                    const common::State &ref_state,
                                    const common::VehicleSet &vehicle_set,
                                    const decimal_t &lat_range,
                                    common::Vehicle *leading_vehicle,
                                    decimal_t *distance_residual_ratio) const;
  ErrorType GetFollowingVehicleOnLane(const common::Lane &ref_lane,
                                      const common::State &ref_state,
                                      const common::VehicleSet &vehicle_set,
                                      const decimal_t &lat_range,
                                      common::Vehicle *leading_vehicle) const;

  /// 查询参考 Lane 上最近前后车，并在存在时输出其 FrenetState 和存在标记。
  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) const;

  /// 以追加 CSV 行的形式记录 ego 与全部周车状态；当前不写表头或地图级元数据。
  ErrorType SaveMapToLog();

  bool IsLocalLaneContainsLane(const int &local_lane_id,
                               const int &seg_lane_id) const;

  ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                          decimal_t *speed_limit) const;

  ErrorType GetTrafficStoppingState(const State &state, const Lane &lane,
                                    State *stopping_state) const;

  /// 使用当前自车状态和空导航路径查询最近 Lane ID。
  ErrorType GetEgoNearestLaneId(int *ego_lane_id) const;

  /// 返回最近一次 UpdateSemanticMap 的输入时间戳。
  inline double time_stamp() const { return time_stamp_; }

  /// 返回当前 ego ID。
  inline int ego_id() const { return ego_id_; }

  /// 返回自车值拷贝。
  inline common::Vehicle ego_vehicle() const { return ego_vehicle_; }

  /// 返回局部障碍物 GridMap 值拷贝。
  inline common::GridMapND<ObstacleMapType, 2> obstacle_map() const {
    return obstacle_map_;
  }
  /// 返回内部 GridMap 的可变裸指针，调用方可直接绕过 setter 修改状态。
  inline common::GridMapND<ObstacleMapType, 2> *obstacle_map_ptr() {
    return &obstacle_map_;
  }
  /// 返回跨帧障碍物世界坐标集合的值拷贝。
  inline std::set<std::array<decimal_t, 2>> obstacle_grids() const {
    return obstacle_grids_;
  }
  /// 返回半径筛选周车集合的值拷贝。
  inline common::VehicleSet surrounding_vehicles() const {
    return surrounding_vehicles_;
  }
  /// 返回关键车辆集合的值拷贝。
  inline common::VehicleSet key_vehicles() const { return key_vehicles_; }

  /// 返回完整 LaneNet 值拷贝。
  inline common::LaneNet whole_lane_net() const { return whole_lane_net_; }

  /// 返回周边 LaneNet 值拷贝。
  inline common::LaneNet surrounding_lane_net() const {
    return surrounding_lane_net_;
  }
  /// 返回语义 LaneSet 值拷贝。
  inline common::SemanticLaneSet semantic_lane_set() const {
    return semantic_lane_set_;
  }
  /// 返回内部语义 LaneSet 的只读裸指针，生命周期隶属本对象。
  inline const common::SemanticLaneSet *semantic_lane_set_cptr() const {
    const common::SemanticLaneSet *ptr = &semantic_lane_set_;
    return ptr;
  }
  /// 返回当前自车 SemanticBehavior 值拷贝。
  inline common::SemanticBehavior ego_behavior() const { return ego_behavior_; }

  /// 返回全部语义周车值拷贝。
  inline common::SemanticVehicleSet semantic_surrounding_vehicles() const {
    return semantic_surrounding_vehicles_;
  }
  /// 返回关键语义车辆值拷贝。
  inline common::SemanticVehicleSet semantic_key_vehicles() const {
    return semantic_key_vehicles_;
  }
  /// 返回 agent 配置值拷贝。
  inline AgentConfigInfo agent_config_info() const {
    return agent_config_info_;
  }
  /// 返回关键车辆 ID 列表值拷贝。
  inline std::vector<int> key_vehicle_ids() const { return key_vehicle_ids_; }

  /// 返回观测噪声模块标记的不确定车辆 ID 值拷贝。
  inline std::vector<int> uncertain_vehicle_ids() const {
    return uncertain_vehicle_ids_;
  }

  /// 返回按车辆 ID 保存的开环状态预测轨迹值拷贝。
  inline std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs()
      const {
    return openloop_pred_trajs_;
  }

  /// 返回 TrafficSignalManager 当前限速列表值拷贝。
  inline vec_E<common::SpeedLimit> RetTrafficInfoSpeedLimit() const {
    return traffic_singal_manager_.speed_limit_list();
  }

  /// 返回 TrafficSignalManager 当前交通灯列表值拷贝。
  inline vec_E<common::TrafficLight> RetTrafficInfoTrafficLight() const {
    return traffic_singal_manager_.traffic_light_list();
  }

  /// 返回本地长 Lane 缓存值拷贝。
  inline std::unordered_map<int, common::Lane> local_lanes() const {
    return local_lanes_;
  }

  /// 直接覆盖 ego ID，不联动刷新其他缓存。
  inline void set_ego_id(const int &in) { ego_id_ = in; }

  /// 深拷贝覆盖局部障碍物 GridMap。
  inline void set_obstacle_map(
      const common::GridMapND<ObstacleMapType, 2> &in) {
    obstacle_map_ = in;
  }
  /// 深拷贝覆盖跨帧障碍物坐标集合。
  inline void set_obstacle_grids(const std::set<std::array<decimal_t, 2>> &in) {
    obstacle_grids_ = in;
  }
  /// 覆盖自车值对象。
  inline void set_ego_vehicle(const common::Vehicle &in) { ego_vehicle_ = in; }

  /// 深拷贝覆盖周车集合。
  inline void set_surrounding_vehicles(const common::VehicleSet &in) {
    surrounding_vehicles_ = in;
  }
  /// 深拷贝覆盖完整 LaneNet。
  inline void set_whole_lane_net(const common::LaneNet &in) {
    whole_lane_net_ = in;
  }
  /// 深拷贝覆盖周边 LaneNet。
  inline void set_surrounding_lane_net(const common::LaneNet &in) {
    surrounding_lane_net_ = in;
  }
  /// 深拷贝覆盖语义 LaneSet，不联动本地 Lane/LUT。
  inline void set_semantic_lane_set(const common::SemanticLaneSet &in) {
    semantic_lane_set_ = in;
  }
  /// 覆盖行为规划器回写的自车 SemanticBehavior。
  inline void set_ego_behavior(const common::SemanticBehavior &in) {
    ego_behavior_ = in;
  }
  /// 覆盖观测噪声模块提供的不确定车辆 ID 列表。
  inline void set_uncertain_vehicle_ids(
      const std::vector<int> &uncertain_vehicle_ids) {
    uncertain_vehicle_ids_ = uncertain_vehicle_ids;
  }

 private:
  /// 把周边 LaneRaw 转为连续 SemanticLane，并裁剪指向当前集合外部的局部拓扑引用。
  ErrorType UpdateSemanticLaneSet();

  /// 围绕自车当前/相邻根 Lane 递归拼接长本地 Lane，并重建 segment/local 双向 LUT。
  ErrorType UpdateLocalLanesAndFastLut();

  /// 为每辆周车匹配最近 Lane、执行 Naive 行为预测并构造对应参考 Lane。
  ErrorType UpdateSemanticVehicles();

  /// 基于自车附近可达 Lane 的近似纵向偏移和前后距离窗口筛选关键周车。
  ErrorType UpdateKeyVehicles();

  /// 清空并重建全部语义周车的开环状态预测轨迹。
  ErrorType OpenloopTrajectoryPrediction();

  ErrorType GetDistanceOnLaneNet(const int &lane_id_0,
                                 const decimal_t &arc_len_0,
                                 const int &lane_id_1,
                                 const decimal_t &arc_len_1,
                                 decimal_t *dist) const;

  /// 在 [s0,s1) 以固定 step 查询 Lane 位置并追加样本，同时对 accum_dist 累加 step。
  ErrorType SampleLane(const common::Lane &lane, const decimal_t &s0,
                       const decimal_t &s1, const decimal_t &step,
                       vec_E<Vecf<2>> *samples, decimal_t *accum_dist) const;

  /// 沿 child_id 递归枚举累计长度达到前向阈值或无子 Lane 的全部路径。
  void GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
      const decimal_t &node_id, const decimal_t &node_length,
      const decimal_t &aggre_length, const std::vector<int> &path_to_node,
      std::vector<std::vector<int>> *all_paths);

  /// 沿 father_id 递归枚举达到后向阈值的路径，并在叶端反转为前向 Lane 顺序。
  void GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
      const decimal_t &node_id, const decimal_t &node_length,
      const decimal_t &aggre_length, const std::vector<int> &path_to_node,
      std::vector<std::vector<int>> *all_paths);

  /// 拼接给定 Lane ID 路径，围绕 state 截取前后长度并重新拟合连续 Lane。
  ErrorType GetLocalLaneUsingLaneIds(const common::State &state,
                                     const std::vector<int> &lane_ids,
                                     const decimal_t max_reflane_dist,
                                     const decimal_t max_backward_dist,
                                     const bool &is_high_quality,
                                     common::Lane *lane);

  /// 根据质量开关选择固定 20-break 正则拟合或直接样本点 Lane 生成。
  ErrorType GetLaneBySampledPoints(const vec_Vecf<2> &samples,
                                   const bool &is_high_quality,
                                   common::Lane *lane) const;

  // 最近地图时间戳，以及固定开环预测时域/步长。
  double time_stamp_{0.0};

  decimal_t pred_time_ = 5.0;
  decimal_t pred_step_ = 0.2;

  // 最近 Lane 搜索和一般 Lane 查询的横向范围参数。
  decimal_t nearest_lane_range_ = 1.5;
  decimal_t lane_range_ = 10.0;

  // 状态到 Lane 的最大允许横向距离。
  decimal_t max_distance_to_lane_ = 2.0;

  // 本地长 Lane 缓存及 segment/local 双向查表。
  bool has_fast_lut_ = false;
  std::unordered_map<int, common::Lane> local_lanes_;
  std::unordered_map<int, std::vector<int>> local_to_segment_lut_;
  std::unordered_map<int, std::set<int>> segment_to_local_lut_;

  // 本地 Lane 前后向目标拼接长度。
  decimal_t local_lane_length_forward_ = 250.0;
  decimal_t local_lane_length_backward_ = 150.0;

  // ego 身份、配置来源/内容及左右轴语义约定。
  int ego_id_;
  std::string agent_config_path_;
  AgentConfigInfo agent_config_info_;
  bool use_right_hand_axis_ = true;

  // 当前原始自车、局部栅格和跨帧障碍坐标。
  common::Vehicle ego_vehicle_;
  GridMap2D obstacle_map_;
  std::set<std::array<decimal_t, 2>> obstacle_grids_;
  // 半径构造的周车及其语义版本。
  common::VehicleSet surrounding_vehicles_;
  common::SemanticVehicleSet semantic_surrounding_vehicles_;
  // 规划相关关键车辆是 surrounding_vehicles_ 的子集，并另存语义对象和 ID。
  common::VehicleSet key_vehicles_;
  common::SemanticVehicleSet semantic_key_vehicles_;
  std::vector<int> key_vehicle_ids_;
  std::vector<int> uncertain_vehicle_ids_;

  // 完整/周边 LaneNet、语义 LaneSet 和行为规划器回写行为。
  common::LaneNet whole_lane_net_;
  common::LaneNet surrounding_lane_net_;
  common::SemanticLaneSet semantic_lane_set_;
  common::SemanticBehavior ego_behavior_;

  // 周车开环轨迹，主要供 on-lane motion planning 碰撞检查使用。
  std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs_;

  // 全局计时器、交通信号管理器和配置加载器裸指针。
  TicToc global_timer_;
  TrafficSignalManager traffic_singal_manager_;
  ConfigLoader *p_config_loader_;

  // 关键车辆筛选等逻辑使用的 RSS 检查器。
  common::RssChecker rss_checker_;

  // 简单 Lane 结构模式只面向 highway-like 拓扑。
  bool is_simple_lane_structure_ = false;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_
