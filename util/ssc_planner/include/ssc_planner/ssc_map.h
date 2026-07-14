/**
 * @file ssc_map.h
 * @author HKUST Aerial Robotics Group
 * @brief 维护 SSC 规划器使用的 Frenet 时空占用地图与驾驶走廊。
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
#define _UTIL_SSC_PLANNER_INC_SSC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "common/basics/semantics.h"
#include "common/state/state.h"

namespace planning {

/// SSC 栅格中复用的障碍物单元数据类型。
using ObstacleMapType = uint8_t;
/// SSC 原始/膨胀占用栅格的数据类型；0 表示空闲，100 表示占用。
using SscMapDataType = uint8_t;

/**
 * @brief 在 Frenet `(s,d,t)` 三维栅格中构建占用和无碰撞驾驶走廊。
 *
 * 类先把静态障碍物沿时间轴拉伸，并把周车多边形轨迹写入离散时间层；随后可按自车尺寸
 * 膨胀障碍物。给定行为层自车初始轨迹后，算法以轨迹栅格坐标为 seed，逐段膨胀轴对齐
 * 立方体形成 DrivingCorridor，最后转换为优化器使用的连续时空语义 cube。
 */
class SscMap {
 public:
  using GridMap3D = common::GridMapND<ObstacleMapType, 3>;

  /// SSC 地图离散化、动力学边界和立方体膨胀参数。
  struct Config {
    std::array<int, 3> map_size = {{1000, 100, 81}};  ///< s/d/t 栅格数量。
    std::array<decimal_t, 3> map_resolution = {
        {0.25, 0.2, 0.1}};  ///< s/d/t 分辨率，单位 m/m/s。
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
    /// 地图原点相对初始纵向位置向后的覆盖长度。
    decimal_t s_back_len = 0.0;

    // 写入最终语义 cube 的纵向速度/加速度统一边界。
    decimal_t kMaxLongitudinalVel = 50.0;
    decimal_t kMinLongitudinalVel = 0.0;
    decimal_t kMaxLongitudinalAcc = 3.0;
    decimal_t kMaxLongitudinalDecel = -8.0;  ///< 常规驾驶员最大减速度。

    // 写入最终语义 cube 的对称横向速度/加速度边界。
    decimal_t kMaxLateralVel = 3.0;
    decimal_t kMaxLateralAcc = 2.5;

    /// 单个走廊 cube 在时间正方向允许覆盖的最大栅格数。
    int kMaxNumOfGridAlongTime = 2;

    /// 六方向膨胀步长，依次为 +s、-s、+d、-d、+t、-t。
    std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};

    /// 将全部配置打印到标准输出，供启动时人工核验。
    void Print() {
      printf("\nSscMap Config:\n");
      printf(" -- map_size: [%d, %d, %d]\n", map_size[0], map_size[1],
             map_size[2]);
      printf(" -- map_resolution: [%lf, %lf, %lf]\n", map_resolution[0],
             map_resolution[1], map_resolution[2]);
      printf(" -- axis_name: [%s, %s, %s]\n", axis_name[0].c_str(),
             axis_name[1].c_str(), axis_name[2].c_str());
      printf(" -- s_back_len: %lf\n", s_back_len);
      printf(" -- kMaxLongitudinalVel: %lf\n", kMaxLongitudinalVel);
      printf(" -- kMinLongitudinalVel: %lf\n", kMinLongitudinalVel);
      printf(" -- kMaxLongitudinalAcc: %lf\n", kMaxLongitudinalAcc);
      printf(" -- kMaxLongitudinalDecel: %lf\n", kMaxLongitudinalDecel);
      printf(" -- kMaxLateralVel: %lf\n", kMaxLateralVel);
      printf(" -- kMaxLateralAcc: %lf\n", kMaxLateralAcc);
      printf(" -- kMaxNumOfGridAlongTime: %d\n", kMaxNumOfGridAlongTime);
      printf(" -- inflate_steps: [%d, %d, %d, %d, %d, %d]\n", inflate_steps[0],
             inflate_steps[1], inflate_steps[2], inflate_steps[3],
             inflate_steps[4], inflate_steps[5]);
    }
  };

  /// 默认构造不分配三维 GridMap，需谨慎避免直接调用地图方法。
  SscMap() {}

  /// 保存配置、打印参数并分别分配原始占用和障碍膨胀三维栅格。
  SscMap(const Config &config);

  /// 当前实现析构为空，不释放构造函数分配的两张 GridMap。
  ~SscMap() {}

  /// 返回可修改的原始占用栅格裸指针。
  GridMap3D *p_3d_grid() const { return p_3d_grid_; }
  /// 返回可修改的自车尺寸膨胀占用栅格裸指针。
  GridMap3D *p_3d_inflated_grid() const { return p_3d_inflated_grid_; }

  /// 按值返回当前配置。
  Config config() const { return config_; }

  /// 按值返回离散栅格坐标下的候选驾驶走廊。
  vec_E<common::DrivingCorridor> driving_corridor_vec() const {
    return driving_corridor_vec_;
  }
  /// 按值返回转换到连续 Frenet 指标后的最终时空 cube 列表。
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec()
      const {
    return final_corridor_vec_;
  };

  /// 返回与候选走廊一一对应的 0/1 有效标志。
  std::vector<int> if_corridor_valid() const { return if_corridor_valid_; }

  /// 保存规划周期起始时间；当前填图逻辑没有直接使用该成员。
  void set_start_time(const decimal_t &t) { start_time_ = t; }
  /// 保存走廊动力学边界和首 cube 校验使用的初始 Frenet 状态。
  void set_initial_fs(const common::FrenetState &fs) { initial_fs_ = fs; }

  /// 按初始 Frenet 状态更新两张三维栅格的 `(s,d,t)` 世界原点。
  void UpdateMapOrigin(const common::FrenetState &ori_fs);

  /// 清空占用并写入静态障碍物和全部周车 Frenet 轨迹。
  ErrorType ConstructSscMap(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs,
      const vec_E<Vec2f> &obstacle_grids);

  /// 按自车前后轴参考点和宽度，将原始占用膨胀到第二张栅格。
  ErrorType InflateObstacleGrid(const common::VehicleParam &param);

  /// 沿给定自车初始轨迹提取 seed、膨胀 cube 并追加一个 DrivingCorridor。
  ErrorType ConstructCorridorUsingInitialTrajectory(
      GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs);

  /// 清空原始占用和膨胀占用两张栅格的数据区。
  ErrorType ClearGridMap();

  /// 清空离散 DrivingCorridor 列表。
  ErrorType ClearDrivingCorridor();

  /// 把所有离散走廊转换为连续 Frenet 位置/速度/加速度/时间约束。
  ErrorType GetFinalGlobalMetricCubesList();

  /// 开始新规划周期：清走廊/栅格、更新时间和地图原点。
  ErrorType ResetSscMap(const common::FrenetState &ini_frenet_state);

 private:
  /// 逐单元检查给定闭区间 cube 内是否全部为 0。
  bool CheckIfCubeIsFree(GridMap3D *p_grid,
                         const common::AxisAlignedCubeNd<int, 3> &cube) const;

  /// 检查固定 s 坐标、覆盖 cube 当前 d/t 范围的平面是否空闲。
  bool CheckIfPlaneIsFreeOnXAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                  const int &z) const;

  /// 检查固定 d 坐标、覆盖 cube 当前 s/t 范围的平面是否空闲。
  bool CheckIfPlaneIsFreeOnYAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                  const int &z) const;

  /// 检查固定 t 坐标、覆盖 cube 当前 s/d 范围的平面是否空闲。
  bool CheckIfPlaneIsFreeOnZAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                  const int &z) const;

  /// 判断三维整数 seed 是否落在 cube 的闭区间边界内。
  bool CheckIfCubeContainsSeed(const common::AxisAlignedCubeNd<int, 3> &cube_a,
                               const Vec3i &seed) const;

  /// 以两个 seed 各维坐标的最小/最大值构造初始轴对齐 cube。
  ErrorType GetInitialCubeUsingSeed(
      const Vec3i &seed_0, const Vec3i &seed_1,
      common::AxisAlignedCubeNd<int, 3> *cube) const;

  /// 从指定 cube 沿序列方向累计时间跨度，收集不小于 t_trans 的相邻下标。
  ErrorType GetTimeCoveredCubeIndices(const common::DrivingCorridor *p_corridor,
                                      const int &start_id, const int &dir,
                                      const int &t_trans,
                                       std::vector<int> *idx_list) const;

  /// 扩张时间相邻 cube 的 s/d 边界，以改善走廊交叠；主流程当前未启用。
  ErrorType CorridorRelaxation(GridMap3D *p_grid,
                                common::DrivingCorridor *p_corridor);

  /// 在动力学可达边界、地图边界和占用约束内按六方向迭代膨胀 cube。
  ErrorType InflateCubeIn3dGrid(GridMap3D *p_grid,
                                const std::array<bool, 6> &dir_disabled,
                                const std::array<int, 6> &dir_step,
                                 common::AxisAlignedCubeNd<int, 3> *cube);

  /// 生成方向禁用表；设计上非首 cube 禁止向负时间扩张。
  ErrorType GetInflationDirections(const bool &if_first_cube,
                                   std::array<bool, 6> *dirs_disabled);

  // 六个单轴膨胀函数每次最多尝试 n_step 层；true 表示已遇边界/障碍而终止。
  bool InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);

  /// 将静态 Frenet 障碍点沿所有离散时间层写入原始占用栅格。
  ErrorType FillStaticPart(const vec_E<Vec2f> &obs_grid_fs);

  /// 逐车把周车 Frenet 多边形轨迹写入对应时间层。
  ErrorType FillDynamicPart(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs);

  /// 将单车轨迹的每个有效车身多边形用 OpenCV fillPoly 栅格化。
  ErrorType FillMapWithFsVehicleTraj(const vec_E<common::FsVehicle> traj);

  // 原始占用栅格与按自车尺寸膨胀后的占用栅格；由带 Config 构造函数 new。
  common::GridMapND<SscMapDataType, 3> *p_3d_grid_;
  common::GridMapND<SscMapDataType, 3> *p_3d_inflated_grid_;

  /// 预留的 cube 相交方向缓存；当前实现没有读写。
  std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;

  Config config_;

  /// 规划周期起始时间；当前只由 setter/Reset 赋值。
  decimal_t start_time_;

  /// 地图原点及走廊动力学边界所依据的初始 Frenet 状态。
  common::FrenetState initial_fs_;

  /// 预留地图有效标志；当前实现始终保持默认 false。
  bool map_valid_ = false;

  /// 栅格坐标系中的候选 DrivingCorridor。
  vec_E<common::DrivingCorridor> driving_corridor_vec_;

  /// 连续指标走廊的有效标志和 cube 列表。
  std::vector<int> if_corridor_valid_;
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec_;
};

}  // namespace planning
#endif  // _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
