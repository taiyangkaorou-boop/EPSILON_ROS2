/**
 * @file semantics.h
 * @author HKUST Aerial Robotics Group
 * @brief 定义车辆、行为、地图、障碍物、SSC 走廊和交通信号等跨模块领域语义。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_SEMANTICS_H_
#define _CORE_COMMON_INC_BASICS_SEMANTICS_H_

#include <assert.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/shapes.h"
#include "common/basics/tool_func.h"
#include "common/lane/lane.h"
#include "common/state/free_state.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"

namespace common {

/**
 * @brief 车辆几何尺寸、轴距和动力学限制参数。
 *
 * 状态坐标默认位于后轴中心，d_cr 描述后轴中心到车辆几何中心的纵向距离。
 * 本类只保存参数，不负责验证物理合理性或单位一致性。
 */
class VehicleParam {
 public:
  /// 返回车身宽度，单位为米。
  inline double width() const { return width_; }
  /// 返回车身总长度，单位为米。
  inline double length() const { return length_; }
  /// 返回前后轴轴距，单位为米。
  inline double wheel_base() const { return wheel_base_; }
  /// 返回前轴到车头的前悬长度，单位为米。
  inline double front_suspension() const { return front_suspension_; }
  /// 返回后轴到车尾的后悬长度，单位为米。
  inline double rear_suspension() const { return rear_suspension_; }
  /// 返回最大转向角；现有配置约定由调用模块解释角度单位。
  inline double max_steering_angle() const { return max_steering_angle_; }
  /// 返回最大纵向加速度绝对限制，单位为 m/s^2。
  inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
  /// 返回最大横向加速度绝对限制，单位为 m/s^2。
  inline double max_lateral_acc() const { return max_lateral_acc_; }
  /// 返回后轴中心到车辆几何中心的纵向距离，单位为米。
  inline double d_cr() const { return d_cr_; }

  /// 设置车身宽度，不执行范围校验。
  inline void set_width(const double val) { width_ = val; }
  /// 设置车身总长度，不执行范围校验。
  inline void set_length(const double val) { length_ = val; }
  /// 设置轴距，不执行范围校验。
  inline void set_wheel_base(const double val) { wheel_base_ = val; }
  /// 设置前悬长度。
  inline void set_front_suspension(const double val) {
    front_suspension_ = val;
  }
  /// 设置后悬长度。
  inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
  /// 设置最大转向角。
  inline void set_max_steering_angle(const double val) {
    max_steering_angle_ = val;
  }
  /// 设置最大纵向加速度限制。
  inline void set_max_longitudinal_acc(const double val) {
    max_longitudinal_acc_ = val;
  }
  /// 设置最大横向加速度限制。
  inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
  /// 设置后轴中心到几何中心的距离。
  inline void set_d_cr(const double val) { d_cr_ = val; }

  /**
   * @brief 输出全部车辆参数，供启动配置和调试核对。
   */
  void print() const;

 private:
  // 默认值对应项目演示车辆；研究实验应通过配置显式记录实际参数。
  double width_ = 1.90;
  double length_ = 4.88;
  double wheel_base_ = 2.85;
  double front_suspension_ = 0.93;
  double rear_suspension_ = 1.10;
  double max_steering_angle_ = 45.0;

  double max_longitudinal_acc_ = 2.0;
  double max_lateral_acc_ = 2.0;

  double d_cr_ = 1.34;  // 车辆几何中心相对后轴中心的纵向偏移。
};

/**
 * @brief 将车辆标识、类别、物理参数和某一时刻运动状态组合为统一对象。
 *
 * Vehicle 不拥有轨迹历史；每个实例表示单个时刻/采样点。其 State 位置以
 * 后轴中心为参考，几何碰撞接口会使用 d_cr 转换到车身几何中心。
 */
class Vehicle {
 public:
  /// 构造 ID 无效、参数和状态为默认值的车辆。
  Vehicle();
  /// 使用车辆参数与状态构造未绑定 ID 的车辆。
  Vehicle(const VehicleParam &param, const State &state);
  /// 使用 ID、参数与状态构造车辆。
  Vehicle(const int &id, const VehicleParam &param, const State &state);
  /// 使用 ID、子类别、参数与状态构造车辆。
  Vehicle(const int &id, const std::string &subclass, const VehicleParam &param,
          const State &state);

  /// 返回车辆唯一 ID。
  inline int id() const { return id_; }
  /// 返回车辆子类别字符串，例如具体车型类别。
  inline std::string subclass() const { return subclass_; }
  /// 按值返回车辆参数副本。
  inline VehicleParam param() const { return param_; }
  /// 按值返回车辆状态副本。
  inline State state() const { return state_; }
  /// 返回车辆主类型字符串。
  inline std::string type() const { return type_; }

  /// 设置车辆唯一 ID。
  inline void set_id(const int &id) { id_ = id; }
  /// 设置车辆子类别。
  inline void set_subclass(const std::string &subclass) {
    subclass_ = subclass;
  }
  /// 设置车辆主类型。
  inline void set_type(const std::string &type) { type_ = type; }
  /// 替换车辆参数。
  inline void set_param(const VehicleParam &in) { param_ = in; }
  /// 替换车辆瞬时状态。
  inline void set_state(const State &in) { state_ = in; }

  /**
   * @brief 返回后轴中心处的二维位姿 `[x, y, yaw]`。
   *
   * @return Vec3f 后轴中心世界坐标和航向角。
   */
  Vec3f Ret3DofState() const;

  /**
   * @brief 返回以车身几何中心为中心的二维有向包围盒。
   *
   * @return OrientedBoundingBox2D 使用车辆宽度、长度和当前航向构造的 OBB。
   */
  OrientedBoundingBox2D RetOrientedBoundingBox() const;

  /**
   * @brief 计算车身矩形的四个世界坐标顶点。
   *
   * @param vertices 输出顶点容器，由 SemanticsUtils 按固定环绕顺序填充。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType RetVehicleVertices(vec_E<Vec2f> *vertices) const;

  /**
   * @brief 返回车身纵向中轴线上的后保险杠点和前保险杠点。
   *
   * @param vertices 输出长度为 2 的数组，索引 0 为后端点、1 为前端点。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType RetBumperVertices(std::array<Vec2f, 2> *vertices) const;

  /**
   * @brief 将后轴中心状态转换为几何中心位姿 `[x, y, yaw]`。
   *
   * @param state 输出三自由度位姿。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType Ret3DofStateAtGeometryCenter(Vec3f *state) const;

  /**
   * @brief 输出车辆 ID、子类别、参数和状态，供调试使用。
   */
  void print() const;

 private:
  int id_{kInvalidAgentId};
  std::string subclass_;
  std::string type_;
  VehicleParam param_;
  State state_;
};

/// 行为层使用的纵向离散意图。
enum class LongitudinalBehavior {
  kMaintain = 0,
  kAccelerate,
  kDecelerate,
  kStopping
};

/// 行为层和预测层共享的横向离散意图。
enum class LateralBehavior {
  kUndefined = 0,
  kLaneKeeping,
  kLaneChangeLeft,
  kLaneChangeRight,
};

/// 使 enum class 可作为 unordered_map 键的哈希适配器。
struct EnumClassHash {
  /// 将枚举底层值转换为 size_t 哈希值。
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

/**
 * @brief 三种有效横向行为的离散概率分布。
 *
 * is_valid 与数值归一化是两个独立条件；设置条目不会自动归一化，也不会自动把
 * is_valid 设为 true。该结构是后续 belief 改造必须兼容的 baseline 概率接口。
 */
struct ProbDistOfLatBehaviors {
  bool is_valid = false;
  std::unordered_map<LateralBehavior, decimal_t, EnumClassHash> probs{
      {common::LateralBehavior::kLaneChangeLeft, 0.0},
      {common::LateralBehavior::kLaneChangeRight, 0.0},
      {common::LateralBehavior::kLaneKeeping, 0.0}};

  /// 写入指定横向行为的概率，不执行截断或归一化。
  void SetEntry(const LateralBehavior &beh, const decimal_t &val) {
    probs[beh] = val;
  }

  /// 检查全部条目之和是否在 kEPS 容差内等于 1。
  bool CheckIfNormalized() const {
    decimal_t sum = 0.0;
    for (const auto &entry : probs) {
      sum += entry.second;
    }
    if (fabs(sum - 1.0) < kEPS) {
      return true;
    } else {
      return false;
    }
  }

  /// 在分布有效时返回概率最大的横向行为；并列结果受哈希遍历顺序影响。
  bool GetMaxProbBehavior(LateralBehavior *beh) const {
    if (!is_valid) return false;

    decimal_t max_prob = -1.0;
    LateralBehavior max_beh;
    for (const auto &entry : probs) {
      if (entry.second > max_prob) {
        max_prob = entry.second;
        max_beh = entry.first;
      }
    }
    *beh = max_beh;
    return true;
  }
};

/**
 * @brief 行为层输出的复合语义决策及其前向仿真证据。
 *
 * ref_lane 可以是由物理车道和离散换道行为重建的参考车道，不要求对应地图中的
 * 单一原始 lane。forward_trajs/forward_behaviors/surround_trajs 保存候选策略评估时
 * 的自车与周车 rollout，state 保存与该语义行为关联的状态。
 */

struct SemanticBehavior {
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  Lane ref_lane;
  decimal_t actual_desired_velocity{0.0};

  vec_E<vec_E<Vehicle>> forward_trajs;
  std::vector<LateralBehavior> forward_behaviors;
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs;

  State state;

  /// 默认构造为车道保持和纵向维持。
  SemanticBehavior() {
    lat_behavior = LateralBehavior::kLaneKeeping;
    lon_behavior = LongitudinalBehavior::kMaintain;
  }
  /// 使用指定横向行为构造；其余成员沿用各自默认值。
  SemanticBehavior(const LateralBehavior &beh) : lat_behavior(beh) {}
};

/**
 * @brief 在 Vehicle 基础上附加车道匹配和横向行为预测信息。
 */
struct SemanticVehicle {
  // 原始车辆参数与瞬时运动状态。
  Vehicle vehicle;

  // 最近车道匹配结果；负值表示尚未完成有效匹配。
  int nearest_lane_id{kInvalidLaneId};
  decimal_t dist_to_lane{-1.0};
  decimal_t arc_len_onlane{-1.0};

  // 横向行为离散概率分布。
  ProbDistOfLatBehaviors probs_lat_behaviors;

  // 当前选取的最大概率行为及其对应参考车道。
  LateralBehavior lat_behavior{LateralBehavior::kUndefined};
  Lane lane;
};

/// 以车辆 ID 为键的语义车辆集合。
struct SemanticVehicleSet {
  std::unordered_map<int, SemanticVehicle> semantic_vehicles;
};

/// 以车辆 ID 为键的原始车辆集合。
struct VehicleSet {
  std::unordered_map<int, Vehicle> vehicles;

  /**
   * @brief 逐车输出 ID 和车辆详细信息。
   */
  void print() const;
};

/**
 * @brief Frenet 坐标系中的车辆状态及其车身顶点。
 */
struct FsVehicle {
  FrenetState frenet_state;
  vec_E<Vec2f> vertices;
};

/**
 * @brief 同时支持开环期望状态和闭环加速度/转向率的车辆控制信号。
 *
 * is_openloop 为 true 时消费 state；为 false 时消费 acc 和 steer_rate。调用方必须
 * 按模式读取字段，不能把两个控制表示同时叠加。
 */
struct VehicleControlSignal {
  double acc = 0.0;
  double steer_rate = 0.0;
  bool is_openloop = false;
  common::State state;

  /**
   * @brief 构造零加速度、零转向率的闭环控制信号。
   */
  VehicleControlSignal();

  /**
   * @brief 构造闭环控制信号。
   *
   * @param acc 纵向加速度，单位 m/s^2。
   * @param steer_rate 转向角速度，单位 rad/s。
   */
  VehicleControlSignal(double acc, double steer_rate);

  /**
   * @brief 构造使用期望状态的开环控制信号。
   *
   * @param state 期望车辆状态。
   */
  VehicleControlSignal(common::State state);
};

/// 以车辆 ID 为键的控制信号集合。
struct VehicleControlSignalSet {
  std::unordered_map<int, VehicleControlSignal> signal_set;
};

struct GridMapMetaInfo {
  int width = 0;
  int height = 0;
  double resolution = 0;
  double w_metric = 0;
  double h_metric = 0;

  /**
   * @brief Construct a new Grid Map Meta Info object
   */
  GridMapMetaInfo();

  /**
   * @brief Construct a new Grid Map Meta Info object
   *
   * @param w width
   * @param h height
   * @param res resolution
   */
  GridMapMetaInfo(const int w, const int h, const double res);

  /**
   * @brief Print info
   */
  void print() const;
};

/**
 * @brief Occupancy grid map
 *
 * @tparam T Data type of the grid map
 * @tparam N_DIM Dimension of the grid map
 */
template <typename T, int N_DIM>
class GridMapND {
 public:
  enum ValType {
    OCCUPIED = 70,
    // FREE = 102,
    FREE = 0,
    SCANNED_OCCUPIED = 128,
    UNKNOWN = 0
  };

  /**
   * @brief Construct a new GridMapND object
   *
   */
  GridMapND();

  /**
   * @brief Construct a new GridMapND object
   *
   * @param dims_size size of each dimension
   * @param dims_resolution resolution of each dimension
   * @param dims_name name of each dimension
   */
  GridMapND(const std::array<int, N_DIM> &dims_size,
            const std::array<decimal_t, N_DIM> &dims_resolution,
            const std::array<std::string, N_DIM> &dims_name);

  inline std::array<int, N_DIM> dims_size() const { return dims_size_; }
  inline int dims_size(const int &dim) const { return dims_size_.at(dim); }
  inline std::array<int, N_DIM> dims_step() const { return dims_step_; }
  inline int dims_step(const int &dim) const { return dims_step_.at(dim); }
  inline std::array<decimal_t, N_DIM> dims_resolution() const {
    return dims_resolution_;
  }
  inline decimal_t dims_resolution(const int &dim) const {
    return dims_resolution_.at(dim);
  }
  inline std::array<std::string, N_DIM> dims_name() const { return dims_name_; }
  inline std::string dims_name(const int &dim) const {
    return dims_name_.at(dim);
  }
  inline std::array<decimal_t, N_DIM> origin() const { return origin_; }
  inline int data_size() const { return data_size_; }
  inline const std::vector<T> *data() const { return &data_; }
  inline T data(const int &i) const { return data_[i]; };
  inline T *get_data_ptr() { return data_.data(); }
  inline const T *data_ptr() const { return data_.data(); }

  inline void set_origin(const std::array<decimal_t, N_DIM> &origin) {
    origin_ = origin;
  }
  inline void set_dims_size(const std::array<int, N_DIM> &dims_size) {
    dims_size_ = dims_size;
    SetNDimSteps(dims_size);
    SetDataSize(dims_size);
  }
  inline void set_dims_resolution(
      const std::array<decimal_t, N_DIM> &dims_resolution) {
    dims_resolution_ = dims_resolution;
  }
  inline void set_dims_name(const std::array<std::string, N_DIM> &dims_name) {
    dims_name_ = dims_name;
  }
  inline void set_data(const std::vector<T> &in) { data_ = in; }

  /**
   * @brief Set all data in array to 0
   */
  inline void clear_data() { data_ = std::vector<T>(data_size_, 0); }

  /**
   * @brief Fill all data in array using value
   *
   * @param val input value
   */
  inline void fill_data(const T &val) {
    data_ = std::vector<T>(data_size_, val);
  }

  /**
   * @brief Get the Value Using Coordinate
   *
   * @param coord Input coordinate
   * @param val Output value
   * @return ErrorType
   */
  ErrorType GetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                    T *val) const;

  /**
   * @brief Get the Value Using Global Position
   *
   * @param p_w Input global position
   * @param val Output value
   * @return ErrorType
   */
  ErrorType GetValueUsingGlobalPosition(const std::array<decimal_t, N_DIM> &p_w,
                                        T *val) const;

  /**
   * @brief Check if the input value is equal to the value in map
   *
   * @param p_w Input global position
   * @param val_in Input value
   * @param res Result
   * @return ErrorType
   */
  ErrorType CheckIfEqualUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w, const T &val_in,
      bool *res) const;

  /**
   * @brief Check if the input value is equal to the value in map
   *
   * @param coord Input coordinate
   * @param val_in Input value
   * @param res Result
   * @return ErrorType
   */
  ErrorType CheckIfEqualUsingCoordinate(const std::array<int, N_DIM> &coord,
                                        const T &val_in, bool *res) const;

  /**
   * @brief Set the Value Using Coordinate
   *
   * @param coord Coordinate of the map
   * @param val Input value
   * @return ErrorType
   */
  ErrorType SetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                    const T &val);

  /**
   * @brief Set the Value Using Global Position
   *
   * @param p_w Global position
   * @param val Input value
   * @return ErrorType
   */
  ErrorType SetValueUsingGlobalPosition(const std::array<decimal_t, N_DIM> &p_w,
                                        const T &val);

  /**
   * @brief Get the Coordinate Using Global Position
   *
   * @param p_w Input global position
   * @return std::array<int, N_DIM> Output coordinate
   */
  std::array<int, N_DIM> GetCoordUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w) const;

  /**
   * @brief Get the Rounded Position Using Global Position object
   *
   * @param p_w Input global position
   * @return std::array<decimal_t, N_DIM> Output global position
   */
  std::array<decimal_t, N_DIM> GetRoundedPosUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w) const;

  /**
   * @brief Get the Global Position Using Coordinate
   *
   * @param coord Input coordinate
   * @param p_w Output global position
   * @return ErrorType
   */
  ErrorType GetGlobalPositionUsingCoordinate(
      const std::array<int, N_DIM> &coord,
      std::array<decimal_t, N_DIM> *p_w) const;

  /**
   * @brief Get the Coordinate Using Global Metric On Single Dimension
   *
   * @param metric Input global 1-dim position
   * @param i Dimension
   * @param idx Output 1-d coordinate
   * @return ErrorType
   */
  ErrorType GetCoordUsingGlobalMetricOnSingleDim(const decimal_t &metric,
                                                 const int &i, int *idx) const;

  /**
   * @brief Get the Global Metric Using Coordinate On Single Dim object
   *
   * @param idx Input 1-d coordinate
   * @param i Dimension
   * @param metric Output 1-d position
   * @return ErrorType
   */
  ErrorType GetGlobalMetricUsingCoordOnSingleDim(const int &idx, const int &i,
                                                 decimal_t *metric) const;

  /**
   * @brief Check if the input coordinate is in map range
   *
   * @param coord Input coordinate
   * @return true In range
   * @return false Out of range
   */
  bool CheckCoordInRange(const std::array<int, N_DIM> &coord) const;

  /**
   * @brief Check if the input 1-d coordinate is in map range
   *
   * @param idx Input 1-d coordinate
   * @param i Dimension
   * @return true In range
   * @return false Out of range
   */
  bool CheckCoordInRangeOnSingleDim(const int &idx, const int &i) const;

  /**
   * @brief Get the mono index using N-dim index
   *
   * @param idx Input N-dim index
   * @return int Output 1-dim index
   */
  int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> &idx) const;

  /**
   * @brief Get N-dim index using mono index
   *
   * @param idx Input mono index
   * @return std::array<int, N_DIM> Output N-dim index
   */
  std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int &idx) const;

 private:
  /**
   * @brief Set the steps of N-dim array
   * @brief E.g. A x-y-z map's steps are {1, x, x*y}
   *
   * @param dims_size Input the size of dimension
   * @return ErrorType
   */
  ErrorType SetNDimSteps(const std::array<int, N_DIM> &dims_size);

  /**
   * @brief Get the Data Size
   *
   * @param dims_size Input the size of dimension
   * @return ErrorType
   */
  ErrorType SetDataSize(const std::array<int, N_DIM> &dims_size);

  std::array<int, N_DIM> dims_size_;
  std::array<int, N_DIM> dims_step_;
  std::array<decimal_t, N_DIM> dims_resolution_;
  std::array<std::string, N_DIM> dims_name_;
  std::array<decimal_t, N_DIM> origin_;

  int data_size_{0};
  std::vector<T> data_;
};

struct LaneRaw {
  int id;
  int dir;

  std::vector<int> child_id;
  std::vector<int> father_id;

  int l_lane_id;
  bool l_change_avbl;
  int r_lane_id;
  bool r_change_avbl;

  std::string behavior;
  decimal_t length;

  Vec2f start_point;
  Vec2f final_point;
  vec_E<Vec2f> lane_points;

  /**
   * @brief Print info
   */
  void print() const;
};

struct LaneNet {
  std::unordered_map<int, LaneRaw> lane_set;

  /**
   * @brief Clear the container
   */
  inline void clear() { lane_set.clear(); }

  /**
   * @brief Print info
   */
  void print() const;
};

struct SemanticLane {
  int id;
  int dir;

  std::vector<int> child_id;
  std::vector<int> father_id;

  int l_lane_id;
  bool l_change_avbl;
  int r_lane_id;
  bool r_change_avbl;

  std::string behavior;
  decimal_t length;

  Lane lane;
};

struct SemanticLaneSet {
  std::unordered_map<int, SemanticLane> semantic_lanes;

  /**
   * @brief Return the size of container
   *
   * @return int size
   */
  inline int size() const { return semantic_lanes.size(); }

  /**
   * @brief Clear the container
   */
  void clear() { semantic_lanes.clear(); }

  /**
   * @brief Print info
   */
  void print() const;
};

struct CircleObstacle {
  int id;
  int type = 0;
  Circle circle;

  /**
   * @brief Print info
   */
  void print() const;
};

struct PolygonObstacle {
  int id;
  int type = 0;
  Polygon polygon;

  /**
   * @brief Print info
   */
  void print() const;
};

struct ObstacleSet {
  std::unordered_map<int, CircleObstacle> obs_circle;
  std::unordered_map<int, PolygonObstacle> obs_polygon;

  /**
   * @brief Return the size of container
   *
   * @return int size
   */
  inline int size() const { return obs_circle.size() + obs_polygon.size(); }

  /**
   * @brief Print info
   */
  void print() const;
};

/**
 * @brief Data structure for Nanoflann
 */
struct PointVecForKdTree {
  std::vector<PointWithValue<int>> pts;

  /**
   * @brief Get the container size
   *
   * @return size_t Output size
   */
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  /**
   * @brief Get value of desired dimension of ith point
   *
   * @param idx Index of point
   * @param dim Dimension
   * @return decimal_t Output value
   */
  inline decimal_t kdtree_get_pt(const size_t idx, const size_t dim) const {
    return dim == 0 ? pts[idx].pt.x : pts[idx].pt.y;
  }

  /**
   * @brief Optional bounding-box computation, not used here
   *
   * @tparam BBOX
   * @return true
   * @return false
   */
  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};

template <int N_DIM>
struct SpatioTemporalSemanticCubeNd {
  decimal_t t_lb, t_ub;
  std::array<decimal_t, N_DIM> p_lb, p_ub;
  std::array<decimal_t, N_DIM> v_lb, v_ub;
  std::array<decimal_t, N_DIM> a_lb, a_ub;

  SpatioTemporalSemanticCubeNd() { FillDefaultBounds(); }

  void FillDefaultBounds() {
    // ~ for optimization, infinity bound will cause numerical
    // ~ unstable. So we put some loose bounds by default
    t_lb = 0.0;
    t_ub = 1.0;

    const decimal_t default_pos_lb = -1;
    const decimal_t default_pos_ub = 1;
    const decimal_t default_vel_lb = -50.0;
    const decimal_t default_vel_ub = 50.0;
    const decimal_t default_acc_lb = -20.0;
    const decimal_t default_acc_ub = 20.0;

    p_lb.fill(default_pos_lb);
    v_lb.fill(default_vel_lb);
    a_lb.fill(default_acc_lb);

    p_ub.fill(default_pos_ub);
    v_ub.fill(default_vel_ub);
    a_ub.fill(default_acc_ub);
  }
};

struct DrivingCube {
  vec_E<Vec3i> seeds;
  AxisAlignedCubeNd<int, 3> cube;
};

struct DrivingCorridor {
  int id;
  bool is_valid;
  vec_E<DrivingCube> cubes;
};

class TrafficSignal {
 public:
  TrafficSignal();
  TrafficSignal(const Vec2f &start_point, const Vec2f &end_point,
                const Vec2f &valid_time, const Vec2f &vel_range);
  void set_start_point(const Vec2f &start_point);
  void set_end_point(const Vec2f &end_point);
  void set_valid_time_til(const decimal_t max_valid_time);
  void set_valid_time_begin(const decimal_t min_valid_time);
  void set_valid_time(const Vec2f &valid_time);
  void set_vel_range(const Vec2f &vel_range);
  void set_lateral_range(const Vec2f &lateral_range);
  void set_max_velocity(const decimal_t max_velocity);

  void set_start_angle(const decimal_t angle) { start_angle_ = angle; }
  void set_end_angle(const decimal_t angle) { end_angle_ = angle; }

  Vec2f start_point() const;
  Vec2f end_point() const;
  Vec2f valid_time() const;
  Vec2f vel_range() const;
  Vec2f lateral_range() const;
  decimal_t max_velocity() const;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  decimal_t start_angle() const { return start_angle_; }
  decimal_t end_angle() const { return end_angle_; }

 protected:
  Vec2f start_point_;          // 2d point in x-y plane
  decimal_t start_angle_ = 0;  // ! (lu.zhang) Temp, for vis
  Vec2f end_point_;            // 2d point in x-y plane
  decimal_t end_angle_ = 0;    // ! (lu.zhang) Temp, for vis
  Vec2f valid_time_;           // time stamp [stamp_begin, stamp_end]
  Vec2f vel_range_;            // velocity [lower_bound, upper_bound]
  Vec2f lateral_range_;
};

class SpeedLimit : public TrafficSignal {
 public:
  SpeedLimit(const Vec2f &start_point, const Vec2f &end_point,
             const Vec2f &vel_range);
};

class StoppingSign : public TrafficSignal {
 public:
  StoppingSign(const Vec2f &start_point, const Vec2f &end_point);
};

class TrafficLight : public TrafficSignal {
 public:
  enum Type { Green = 0, Red, Yellow, RedYellow };
  void set_type(const Type &type);
  Type type() const;

 private:
  Type type_;
};

class SemanticsUtils {
 public:
  /**
   * @brief Return the name of longitudinal behavior
   *
   * @param behavior
   * @return std::string
   */
  static std::string RetLonBehaviorName(const LongitudinalBehavior b) {
    std::string b_str;
    switch (b) {
      case LongitudinalBehavior::kMaintain: {
        b_str = std::string("M");
        break;
      }
      case LongitudinalBehavior::kAccelerate: {
        b_str = std::string("A");
        break;
      }
      case LongitudinalBehavior::kDecelerate: {
        b_str = std::string("D");
        break;
      }
      case LongitudinalBehavior::kStopping: {
        b_str = std::string("S");
        break;
      }
      default: {
        b_str = std::string("Null");
        break;
      }
    }
    return b_str;
  }

  /**
   * @brief Return the name of lateral behavior
   *
   * @param behavior
   * @return std::string
   */
  static std::string RetLatBehaviorName(const LateralBehavior b) {
    std::string b_str;
    switch (b) {
      case LateralBehavior::kUndefined: {
        b_str = std::string("U");
        break;
      }
      case LateralBehavior::kLaneKeeping: {
        b_str = std::string("K");
        break;
      }
      case LateralBehavior::kLaneChangeLeft: {
        b_str = std::string("L");
        break;
      }
      case LateralBehavior::kLaneChangeRight: {
        b_str = std::string("R");
        break;
      }
      default: {
        b_str = std::string("Null");
        break;
      }
    }
    return b_str;
  }

  /**
   * @brief Get the Oriented Bounding Box For Vehicle Using State object
   *
   * @param param Vehicle parameter
   * @param s Vehicle state
   * @param obb Output Vehicle OBB
   * @return ErrorType
   */
  static ErrorType GetOrientedBoundingBoxForVehicleUsingState(
      const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb);

  /**
   * @brief Get the Vehicle Vertices
   *
   * @param param Vehicle parameter
   * @param state Vehicle state
   * @param vertices Output vertice vector
   * @return ErrorType
   */
  static ErrorType GetVehicleVertices(const VehicleParam &param,
                                      const State &state,
                                      vec_E<Vec2f> *vertices);

  static ErrorType InflateVehicleBySize(const Vehicle &vehicle_in,
                                        const decimal_t delta_w,
                                        const decimal_t delta_l,
                                        Vehicle *vehicle_out);

};  // SemanticsUtils

}  // namespace common

#endif  // _CORE_COMMON_INC_BASICS_SEMANTICS_H_
