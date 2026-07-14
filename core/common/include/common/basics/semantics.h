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

/// 二维栅格地图的尺寸、分辨率和物理覆盖范围元数据。
struct GridMapMetaInfo {
  int width = 0;
  int height = 0;
  double resolution = 0;
  double w_metric = 0;
  double h_metric = 0;

  /**
   * @brief 构造全部字段为零的空元数据。
   */
  GridMapMetaInfo();

  /**
   * @brief 使用栅格宽高和统一分辨率构造二维地图元数据。
   *
   * @param w x/宽度方向栅格数量。
   * @param h y/高度方向栅格数量。
   * @param res 每个栅格的物理分辨率。
   */
  GridMapMetaInfo(const int w, const int h, const double res);

  /**
   * @brief 输出栅格数量、分辨率和物理尺寸，供调试使用。
   */
  void print() const;
};

/**
 * @brief 使用一维连续数组存储的 N 维规则栅格地图。
 *
 * 第 0 维步长为 1，随后各维步长为之前维度尺寸的累乘，因此第 0 维在内存中
 * 连续变化最快。世界位置与栅格坐标之间使用 round 而不是 floor 转换。
 *
 * @tparam T 单栅格数据类型。
 * @tparam N_DIM 栅格维数。
 */
template <typename T, int N_DIM>
class GridMapND {
 public:
  /// 传统占据栅格值；FREE 与 UNKNOWN 当前同为 0，调用方无法仅凭数值区分二者。
  enum ValType {
    OCCUPIED = 70,
    // FREE = 102,
    FREE = 0,
    SCANNED_OCCUPIED = 128,
    UNKNOWN = 0
  };

  /**
   * @brief 默认构造空地图；尺寸、分辨率和原点需由调用方继续设置。
   *
   */
  GridMapND();

  /**
   * @brief 使用各维尺寸、分辨率和名称构造零填充地图，原点为全零。
   *
   * @param dims_size 各维栅格数量。
   * @param dims_resolution 各维物理分辨率，必须为非零正值。
   * @param dims_name 各维语义名称，例如 x、y、t。
   */
  GridMapND(const std::array<int, N_DIM> &dims_size,
            const std::array<decimal_t, N_DIM> &dims_resolution,
            const std::array<std::string, N_DIM> &dims_name);

  /// 返回全部维度尺寸副本。
  inline std::array<int, N_DIM> dims_size() const { return dims_size_; }
  /// 返回指定维度尺寸，越界时由 std::array::at 抛出异常。
  inline int dims_size(const int &dim) const { return dims_size_.at(dim); }
  /// 返回 N 维坐标转一维索引使用的步长数组。
  inline std::array<int, N_DIM> dims_step() const { return dims_step_; }
  /// 返回指定维度索引步长，越界时抛出异常。
  inline int dims_step(const int &dim) const { return dims_step_.at(dim); }
  /// 返回全部维度物理分辨率副本。
  inline std::array<decimal_t, N_DIM> dims_resolution() const {
    return dims_resolution_;
  }
  /// 返回指定维度物理分辨率，越界时抛出异常。
  inline decimal_t dims_resolution(const int &dim) const {
    return dims_resolution_.at(dim);
  }
  /// 返回全部维度名称副本。
  inline std::array<std::string, N_DIM> dims_name() const { return dims_name_; }
  /// 返回指定维度名称，越界时抛出异常。
  inline std::string dims_name(const int &dim) const {
    return dims_name_.at(dim);
  }
  /// 返回各维坐标 0 对应的世界原点副本。
  inline std::array<decimal_t, N_DIM> origin() const { return origin_; }
  /// 返回尺寸累乘得到的理论数据元素数量。
  inline int data_size() const { return data_size_; }
  /// 返回底层数据 vector 的只读指针；对象生命周期内有效。
  inline const std::vector<T> *data() const { return &data_; }
  /// 按一维下标返回数据，不执行边界检查。
  inline T data(const int &i) const { return data_[i]; };
  /// 返回可写底层连续数据指针，调用方必须保证不越过 data_.size()。
  inline T *get_data_ptr() { return data_.data(); }
  /// 返回只读底层连续数据指针。
  inline const T *data_ptr() const { return data_.data(); }

  /// 设置地图世界原点，不移动或重采样已有数据。
  inline void set_origin(const std::array<decimal_t, N_DIM> &origin) {
    origin_ = origin;
  }
  /// 更新维度尺寸、索引步长和理论 data_size；当前不会同步调整 data_ 长度。
  inline void set_dims_size(const std::array<int, N_DIM> &dims_size) {
    dims_size_ = dims_size;
    SetNDimSteps(dims_size);
    SetDataSize(dims_size);
  }
  /// 设置各维分辨率，不重投影已有数据。
  inline void set_dims_resolution(
      const std::array<decimal_t, N_DIM> &dims_resolution) {
    dims_resolution_ = dims_resolution;
  }
  /// 设置各维语义名称。
  inline void set_dims_name(const std::array<std::string, N_DIM> &dims_name) {
    dims_name_ = dims_name;
  }
  /// 直接替换底层数据，不检查输入长度是否等于 data_size_。
  inline void set_data(const std::vector<T> &in) { data_ = in; }

  /**
   * @brief 按理论 data_size_ 重新分配底层数组并全部置零。
   */
  inline void clear_data() { data_ = std::vector<T>(data_size_, 0); }

  /**
   * @brief 按理论 data_size_ 重新分配底层数组并填充指定值。
   *
   * @param val 每个栅格写入的值。
   */
  inline void fill_data(const T &val) {
    data_ = std::vector<T>(data_size_, val);
  }

  /**
   * @brief 使用 N 维离散坐标读取栅格值。
   *
   * @param coord 输入离散坐标。
   * @param val 输出栅格值。
   * @return ErrorType 坐标有效返回 kSuccess，否则返回 kWrongStatus。
   */
  ErrorType GetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                    T *val) const;

  /**
   * @brief 将世界位置四舍五入到最近栅格后读取值。
   *
   * @param p_w 输入世界位置。
   * @param val 输出栅格值。
   * @return ErrorType 当前实现忽略内部越界状态并固定返回 kSuccess。
   */
  ErrorType GetValueUsingGlobalPosition(const std::array<decimal_t, N_DIM> &p_w,
                                        T *val) const;

  /**
   * @brief 判断世界位置对应栅格是否等于给定值。
   *
   * @param p_w 输入世界位置。
   * @param val_in 待比较值。
   * @param res 输出比较结果；越界时为 false。
   * @return ErrorType 当前实现固定返回 kSuccess，越界通过 res=false 表达。
   */
  ErrorType CheckIfEqualUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w, const T &val_in,
      bool *res) const;

  /**
   * @brief 判断离散坐标对应栅格是否等于给定值。
   *
   * @param coord 输入离散坐标。
   * @param val_in 待比较值。
   * @param res 输出比较结果；越界时为 false。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType CheckIfEqualUsingCoordinate(const std::array<int, N_DIM> &coord,
                                        const T &val_in, bool *res) const;

  /**
   * @brief 使用 N 维离散坐标写入栅格值。
   *
   * @param coord 输入离散坐标。
   * @param val 待写入值。
   * @return ErrorType 坐标有效返回 kSuccess，否则返回 kWrongStatus。
   */
  ErrorType SetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                    const T &val);

  /**
   * @brief 将世界位置四舍五入到最近栅格后写入值。
   *
   * @param p_w 输入世界位置。
   * @param val 待写入值。
   * @return ErrorType 当前实现忽略内部越界状态并固定返回 kSuccess。
   */
  ErrorType SetValueUsingGlobalPosition(const std::array<decimal_t, N_DIM> &p_w,
                                        const T &val);

  /**
   * @brief 将世界位置按各维分辨率四舍五入为离散坐标。
   *
   * @param p_w 输入世界位置。
   * @return std::array<int, N_DIM> 离散坐标；不保证位于地图范围内。
   */
  std::array<int, N_DIM> GetCoordUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w) const;

  /**
   * @brief 将世界位置吸附到最近栅格中心对应的世界位置。
   *
   * @param p_w 输入世界位置。
   * @return std::array<decimal_t, N_DIM> 最近离散坐标对应的世界位置。
   */
  std::array<decimal_t, N_DIM> GetRoundedPosUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w) const;

  /**
   * @brief 将离散坐标转换为世界位置。
   *
   * @param coord 输入离散坐标；当前实现不检查范围。
   * @param p_w 输出世界位置。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType GetGlobalPositionUsingCoordinate(
      const std::array<int, N_DIM> &coord,
      std::array<decimal_t, N_DIM> *p_w) const;

  /**
   * @brief 将单维世界坐标四舍五入为该维离散下标。
   *
   * @param metric 输入单维世界坐标。
   * @param i 维度下标。
   * @param idx 输出离散下标；不检查是否在范围内。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType GetCoordUsingGlobalMetricOnSingleDim(const decimal_t &metric,
                                                 const int &i, int *idx) const;

  /**
   * @brief 将单维离散下标转换为世界坐标。
   *
   * @param idx 输入离散下标。
   * @param i 维度下标。
   * @param metric 输出世界坐标。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType GetGlobalMetricUsingCoordOnSingleDim(const int &idx, const int &i,
                                                 decimal_t *metric) const;

  /**
   * @brief 检查 N 维坐标的每个分量是否位于 `[0, dims_size)`。
   *
   * @param coord 输入离散坐标。
   * @return true 所有维度均有效。
   * @return false 至少一维越界。
   */
  bool CheckCoordInRange(const std::array<int, N_DIM> &coord) const;

  /**
   * @brief 检查单维离散下标是否有效。
   *
   * @param idx 输入离散下标。
   * @param i 维度下标；调用方必须保证 i 有效。
   * @return true 下标位于 `[0, dims_size[i])`。
   * @return false 下标越界。
   */
  bool CheckCoordInRangeOnSingleDim(const int &idx, const int &i) const;

  /**
   * @brief 使用预计算步长把 N 维下标展开为一维下标。
   *
   * @param idx 输入 N 维下标；当前实现不检查范围。
   * @return int 底层 data_ 的一维下标。
   */
  int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> &idx) const;

  /**
   * @brief 使用步长除法把一维下标还原为 N 维下标。
   *
   * @param idx 输入一维下标；当前实现不检查范围。
   * @return std::array<int, N_DIM> 还原后的 N 维下标。
   */
  std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int &idx) const;

 private:
  /**
   * @brief 根据各维尺寸计算一维展开步长，例如 x-y-z 为 `{1, x, x*y}`。
   *
   * @param dims_size 各维栅格数量。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType SetNDimSteps(const std::array<int, N_DIM> &dims_size);

  /**
   * @brief 计算各维尺寸乘积并保存为理论数据元素数量。
   *
   * @param dims_size 各维栅格数量。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  ErrorType SetDataSize(const std::array<int, N_DIM> &dims_size);

  // 网格结构元数据：尺寸、线性步长、分辨率、维度名称和世界原点。
  std::array<int, N_DIM> dims_size_;
  std::array<int, N_DIM> dims_step_;
  std::array<decimal_t, N_DIM> dims_resolution_;
  std::array<std::string, N_DIM> dims_name_;
  std::array<decimal_t, N_DIM> origin_;

  // data_size_ 是尺寸乘积；data_ 的实际长度可被 set_data 独立改变。
  int data_size_{0};
  std::vector<T> data_;
};

/**
 * @brief 从场景配置直接解析得到的原始车道记录。
 *
 * child/father 描述纵向拓扑，左右车道 ID 与换道可用标记描述横向拓扑，lane_points
 * 保存离散中心线。该结构不提供几何插值，需转换为 SemanticLane/Lane 后使用。
 */
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
   * @brief 输出拓扑、换道属性、端点和离散点数量，供地图解析调试。
   */
  void print() const;
};

/// 以车道 ID 为键的原始车道网络。
struct LaneNet {
  std::unordered_map<int, LaneRaw> lane_set;

  /**
   * @brief 清空全部原始车道记录。
   */
  inline void clear() { lane_set.clear(); }

  /**
   * @brief 输出车道数量并逐车道打印详细信息。
   */
  void print() const;
};

/**
 * @brief 完成几何拟合后的语义车道。
 *
 * 保留 LaneRaw 的拓扑和行为属性，并以 Lane 对象替代离散 lane_points，供投影、
 * Frenet 转换和规划器查询使用。
 */
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

/// 以车道 ID 为键的语义车道集合。
struct SemanticLaneSet {
  std::unordered_map<int, SemanticLane> semantic_lanes;

  /**
   * @brief 返回语义车道数量。
   *
   * @return int 容器元素数量。
   */
  inline int size() const { return semantic_lanes.size(); }

  /**
   * @brief 清空全部语义车道。
   */
  void clear() { semantic_lanes.clear(); }

  /**
   * @brief 输出语义车道集合的数量摘要。
   */
  void print() const;
};

/// 带 ID 和类型码的圆形静态障碍物。
struct CircleObstacle {
  int id;
  int type = 0;
  Circle circle;

  /**
   * @brief 输出障碍物 ID 和圆几何信息。
   */
  void print() const;
};

/// 带 ID 和类型码的多边形静态障碍物。
struct PolygonObstacle {
  int id;
  int type = 0;
  Polygon polygon;

  /**
   * @brief 输出障碍物 ID 和多边形几何信息。
   */
  void print() const;
};

/// 分别按 ID 保存圆形与多边形障碍物的集合。
struct ObstacleSet {
  std::unordered_map<int, CircleObstacle> obs_circle;
  std::unordered_map<int, PolygonObstacle> obs_polygon;

  /**
   * @brief 返回两类障碍物数量之和。
   *
   * @return int 圆形和多边形障碍物总数。
   */
  inline int size() const { return obs_circle.size() + obs_polygon.size(); }

  /**
   * @brief 依次输出全部圆形和多边形障碍物。
   */
  void print() const;
};

/**
 * @brief 为 nanoflann 提供二维点云访问协议的数据适配器。
 *
 * 每个点可携带整数属性数组；KD-tree 只读取 Point 的 x、y，不访问附加 values。
 */
struct PointVecForKdTree {
  std::vector<PointWithValue<int>> pts;

  /**
   * @brief 返回 KD-tree 可索引点数量。
   *
   * @return size_t pts 容器大小。
   */
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  /**
   * @brief 返回第 idx 个点在指定维度上的坐标。
   *
   * @param idx 点下标，调用方必须保证不越界。
   * @param dim 维度；0 返回 x，其他任意值均返回 y。
   * @return decimal_t 对应二维坐标值。
   */
  inline decimal_t kdtree_get_pt(const size_t idx, const size_t dim) const {
    return dim == 0 ? pts[idx].pt.x : pts[idx].pt.y;
  }

  /**
   * @brief 告知 nanoflann 本适配器不提供预计算包围盒。
   *
   * @tparam BBOX nanoflann 请求的包围盒类型。
   * @return false 始终要求 nanoflann 自行计算包围盒。
   */
  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};

/**
 * @brief 单个时间段内对位置、速度和加速度施加上下界的时空语义 cube。
 *
 * cube 是 SSC/QP 的约束载体。默认边界使用有限宽松值而非无穷大，以避免数值求解
 * 不稳定；调用方应在加入 corridor 前覆盖真实时间与运动边界。
 */
template <int N_DIM>
struct SpatioTemporalSemanticCubeNd {
  decimal_t t_lb, t_ub;
  std::array<decimal_t, N_DIM> p_lb, p_ub;
  std::array<decimal_t, N_DIM> v_lb, v_ub;
  std::array<decimal_t, N_DIM> a_lb, a_ub;

  /// 构造并填充宽松有限默认边界。
  SpatioTemporalSemanticCubeNd() { FillDefaultBounds(); }

  /// 重置时间、位置、速度和加速度边界为数值稳定的默认范围。
  void FillDefaultBounds() {
    // 优化器使用无穷边界可能数值不稳定，因此使用有限但宽松的默认约束。
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

/// SSC 栅格中的一个三维 driving cube 及其生成时使用的种子体素。
struct DrivingCube {
  vec_E<Vec3i> seeds;
  AxisAlignedCubeNd<int, 3> cube;
};

/// 一条由有序 driving cube 构成的候选时空走廊。
struct DrivingCorridor {
  int id;
  bool is_valid;
  vec_E<DrivingCube> cubes;
};

/**
 * @brief 以二维线段为作用区域、带时间/速度/横向范围约束的交通语义基类。
 *
 * start/end angle 目前只服务可视化；真正约束范围由端点、valid_time、vel_range 和
 * lateral_range 表达。所有 setter 均为直接赋值，不检查上下界顺序。
 */
class TrafficSignal {
 public:
  /// 构造零长度线段、全时间有效、零速度范围和默认半车道横向范围的信号。
  TrafficSignal();
  /// 使用作用线段、有效时间和速度范围构造信号，横向范围使用默认值。
  TrafficSignal(const Vec2f &start_point, const Vec2f &end_point,
                 const Vec2f &valid_time, const Vec2f &vel_range);
  /// 设置作用线段起点。
  void set_start_point(const Vec2f &start_point);
  /// 设置作用线段终点。
  void set_end_point(const Vec2f &end_point);
  /// 只设置有效时间区间上界。
  void set_valid_time_til(const decimal_t max_valid_time);
  /// 只设置有效时间区间下界。
  void set_valid_time_begin(const decimal_t min_valid_time);
  /// 同时设置有效时间 `[begin, end]`。
  void set_valid_time(const Vec2f &valid_time);
  /// 设置允许速度区间 `[min, max]`。
  void set_vel_range(const Vec2f &vel_range);
  /// 设置相对信号线的允许横向范围。
  void set_lateral_range(const Vec2f &lateral_range);
  /// 只设置速度区间上界。
  void set_max_velocity(const decimal_t max_velocity);

  /// 设置起点方向角，仅供当前可视化使用。
  void set_start_angle(const decimal_t angle) { start_angle_ = angle; }
  /// 设置终点方向角，仅供当前可视化使用。
  void set_end_angle(const decimal_t angle) { end_angle_ = angle; }

  /// 返回作用线段起点。
  Vec2f start_point() const;
  /// 返回作用线段终点。
  Vec2f end_point() const;
  /// 返回有效时间区间。
  Vec2f valid_time() const;
  /// 返回允许速度区间。
  Vec2f vel_range() const;
  /// 返回允许横向范围。
  Vec2f lateral_range() const;
  /// 返回速度区间上界，与 vel_range()(1) 等价。
  decimal_t max_velocity() const;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /// 返回可视化起点方向角。
  decimal_t start_angle() const { return start_angle_; }
  /// 返回可视化终点方向角。
  decimal_t end_angle() const { return end_angle_; }

 protected:
  Vec2f start_point_;          // x-y 平面中的作用线段起点。
  decimal_t start_angle_ = 0;  // 临时可视化角度，不参与约束计算。
  Vec2f end_point_;            // x-y 平面中的作用线段终点。
  decimal_t end_angle_ = 0;    // 临时可视化角度，不参与约束计算。
  Vec2f valid_time_;           // 有效时间戳区间 [begin, end]。
  Vec2f vel_range_;            // 允许速度区间 [lower, upper]。
  Vec2f lateral_range_;
};

/// 在指定线段区域施加速度上下界的交通语义。
class SpeedLimit : public TrafficSignal {
 public:
  /// 构造全时间有效的限速区域。
  SpeedLimit(const Vec2f &start_point, const Vec2f &end_point,
             const Vec2f &vel_range);
};

/// 在指定停止线区域施加零速度要求的交通语义。
class StoppingSign : public TrafficSignal {
 public:
  /// 构造全时间有效、速度范围为零的停止标志。
  StoppingSign(const Vec2f &start_point, const Vec2f &end_point);
};

/// 带离散灯色状态的交通信号灯语义。
class TrafficLight : public TrafficSignal {
 public:
  /// 信号灯可用状态；RedYellow 表示红黄同时点亮的过渡状态。
  enum Type { Green = 0, Red, Yellow, RedYellow };
  /// 设置当前灯色；默认构造后调用方必须显式赋值。
  void set_type(const Type &type);
  /// 返回当前灯色；未显式设置时成员值未初始化。
  Type type() const;

 private:
  Type type_;
};

/**
 * @brief 与语义对象相关的无状态转换和车辆几何辅助函数。
 *
 * 本类不缓存车辆或地图；所有结果通过返回值/输出指针交付。几何计算仍以 State 的
 * 后轴中心为输入，并通过 VehicleParam::d_cr 转换到几何中心。
 */
class SemanticsUtils {
 public:
  /**
   * @brief 将纵向行为映射为日志使用的单字符缩写。
   *
   * @param b 纵向行为枚举。
   * @return std::string M/A/D/S；未知枚举返回 "Null"。
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
   * @brief 将横向行为映射为日志使用的单字符缩写。
   *
   * @param b 横向行为枚举。
   * @return std::string U/K/L/R；未知枚举返回 "Null"。
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
   * @brief 根据车辆参数和后轴中心状态构造车身 OBB。
   *
   * @param param 车辆几何参数。
   * @param s 后轴中心处的车辆状态。
   * @param obb 输出以几何中心为中心的 OBB。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetOrientedBoundingBoxForVehicleUsingState(
      const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb);

  /**
   * @brief 计算车身四个顶点并按左前开始逆时针追加到输出容器。
   *
   * @param param 车辆几何参数。
   * @param state 后轴中心处的车辆状态。
   * @param vertices 输出顶点容器；当前实现不会先清空已有内容。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetVehicleVertices(const VehicleParam &param,
                                      const State &state,
                                      vec_E<Vec2f> *vertices);

  /**
   * @brief 在不改变 ID、类别和状态的前提下扩张车辆宽度与长度。
   * @param vehicle_in 输入车辆。
   * @param delta_w 宽度增量，可为负但当前不检查结果是否为正。
   * @param delta_l 长度增量，可为负但当前不检查结果是否为正。
   * @param vehicle_out 输出扩张后的车辆副本。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType InflateVehicleBySize(const Vehicle &vehicle_in,
                                        const decimal_t delta_w,
                                        const decimal_t delta_l,
                                        Vehicle *vehicle_out);

};  // SemanticsUtils

}  // namespace common

#endif  // _CORE_COMMON_INC_BASICS_SEMANTICS_H_
