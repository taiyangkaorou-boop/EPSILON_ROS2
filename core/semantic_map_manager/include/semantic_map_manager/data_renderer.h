#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_

#include <random>
#include <assert.h>
#include <iostream>
#include <set>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "nanoflann/include/nanoflann.hpp"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

/**
 * @brief 把原始 LaneNet、车辆集和障碍物集渲染为单个 ego 的局部语义地图输入。
 *
 * Renderer 从 SemanticMapManager 配置获取 ego ID、栅格规格和搜索半径；每帧提取自车、
 * 局部障碍物栅格、周边 Lane/车辆，可选注入观测噪声并执行射线可见性模拟，最后回写 SMM。
 */
class DataRenderer {
 public:
  using ObstacleMapType = uint8_t;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  /// 绑定非拥有 SMM 指针，读取 agent 配置并动态创建局部障碍物 GridMap。
  DataRenderer(SemanticMapManager *smm_ptr);

  /// 当前为空析构；构造函数 new 的障碍物 GridMap 不会在此释放。
  ~DataRenderer() {}

  /// 覆盖后续从 VehicleSet 中查找的 ego ID。
  inline void set_ego_id(const int id) { ego_id_ = id; }

  /// 只覆盖保存的 GridMap 元信息，不重新分配已经创建的栅格对象。
  inline void set_obstacle_map_info(const common::GridMapMetaInfo &info) {
    obstacle_map_info_ = info;
  }

  /// 执行一帧完整数据渲染并调用 SemanticMapManager::UpdateSemanticMap。
  ErrorType Render(const double &time_stamp, const common::LaneNet &lane_net,
                   const common::VehicleSet &vehicle_set,
                   const common::ObstacleSet &obstacle_set);

 private:
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<decimal_t, common::PointVecForKdTree>,
      common::PointVecForKdTree, 2>
      KdTreeFor2dPointVec;

  /// 按 ego_id_ 从 VehicleSet 复制自车及其参数和状态缓存。
  ErrorType GetEgoVehicle(const common::VehicleSet &vehicle_set);

  /// 以自车为中心重置局部 GridMap，并用 OpenCV 栅格化圆形/多边形静态障碍。
  ErrorType GetObstacleMap(const common::ObstacleSet &obstacle_set);

  /// 保存输入完整 LaneNet 的值拷贝。
  ErrorType GetWholeLaneNet(const common::LaneNet &lane_net);

  /// 为全部 Lane 采样点重建 KD-tree，并提取自车附近命中的 LaneRaw。
  ErrorType GetSurroundingLaneNet(const common::LaneNet &lane_net);

  /// 按欧氏距离筛选搜索半径内的非 ego 车辆。
  ErrorType GetSurroundingVehicles(const common::VehicleSet &vehicle_set);

  /// 预留周边对象聚合入口；当前只有声明，未被调用且没有实现定义。
  ErrorType GetSurroundingObjects();

  /// 从自车几何中心向八个象限执行栅格 FOV 光线投射并更新可见障碍记忆。
  ErrorType RayCastingOnObstacleMap();

  /// 运行射线投射，保留附近历史障碍并把它们标为 SCANNED_OCCUPIED。
  ErrorType FakeMapper();

  /// 每 10 帧为 ego 0 随机选择最多三辆周车注入位置/航向高斯噪声。
  ErrorType InjectObservationNoise();

  // Lane 点云与 KD-tree；updated 标记当前未参与任何复用判断。
  bool if_kdtree_lane_net_updated_ = false;
  common::PointVecForKdTree lane_net_pts_;
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_lane_net_;

  // 障碍物点云与 KD-tree 预留成员；当前渲染路径未使用。
  bool if_kdtree_obstacle_set_updated_ = false;
  common::PointVecForKdTree obstacle_set_pts_;
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_obstacle_set_;

  // 车辆点云与 KD-tree 预留成员；周车筛选当前使用线性遍历。
  common::PointVecForKdTree vehicle_set_pts_;
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_vehicle_;

  int ego_id_ = 0;

  // 预留射线数量；实际实现固定调用八个 FOV octant，并未读取该值。
  int ray_casting_num_ = 1440;

  // 跨帧保留的已观测障碍世界坐标，以及尚未使用的自由栅格集合。
  std::set<std::array<decimal_t, 2>> obs_grids_;
  std::set<std::array<decimal_t, 2>> free_grids_;

  double time_stamp_;

  // 当前帧自车及其参数/状态缓存。
  common::Vehicle ego_vehicle_;
  common::VehicleParam ego_param_;
  common::State ego_state_;

  // 当前帧半径筛选后的周车集合。
  common::VehicleSet surrounding_vehicles_;

  // 局部栅格配置及动态分配的工作栅格裸指针。
  common::GridMapMetaInfo obstacle_map_info_;
  GridMap2D *p_obstacle_grid_;

  decimal_t surrounding_search_radius_;

  // 当前帧局部 LaneNet 与输入完整 LaneNet 的值拷贝。
  common::LaneNet surrounding_lane_net_;
  common::LaneNet whole_lane_net_;

  // 非拥有 SemanticMapManager 回写目标。
  SemanticMapManager *p_semantic_map_manager_;

  // 跟踪噪声模拟状态：不确定车辆 ID、默认种子的随机引擎和十帧计数器。
  std::vector<int> uncertain_vehicle_ids_;
  std::mt19937 random_engine_;
  int cnt_random_ = 0;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
