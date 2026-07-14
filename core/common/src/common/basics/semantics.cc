/**
 * @file semantics.cc
 * @author HKUST Aerial Robotics Group
 * @brief 实现语义数据对象的几何转换、调试输出、栅格访问和交通信号操作。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/semantics.h"

namespace common {

void VehicleParam::print() const {
  // 按配置字段逐项输出，便于确认车辆模型与实验记录是否一致。
  printf("VehicleParam:\n");
  printf(" -- width:\t %lf.\n", width_);
  printf(" -- length:\t %lf.\n", length_);
  printf(" -- wheel_base:\t %lf.\n", wheel_base_);
  printf(" -- front_suspension:\t %lf.\n", front_suspension_);
  printf(" -- rear_suspension:\t %lf.\n", rear_suspension_);
  printf(" -- d_cr:\t %lf.\n", d_cr_);
  printf(" -- max_steering_angle:\t %lf.\n", max_steering_angle_);
  printf(" -- max_longitudinal_acc:\t %lf.\n", max_longitudinal_acc_);
  printf(" -- max_lateral_acc:\t %lf.\n", max_lateral_acc_);
}

// 构造函数仅组合 ID、类别、参数和状态，不执行几何或动力学校验。
Vehicle::Vehicle() {}

Vehicle::Vehicle(const VehicleParam &param, const State &state)
    : param_(param), state_(state) {}

Vehicle::Vehicle(const int &id, const VehicleParam &param, const State &state)
    : id_(id), param_(param), state_(state) {}

Vehicle::Vehicle(const int &id, const std::string &subclass,
                 const VehicleParam &param, const State &state)
    : id_(id), subclass_(subclass), param_(param), state_(state) {}

Vec3f Vehicle::Ret3DofState() const {
  // State 的位置参考点为后轴中心，因此直接读取位置和航向。
  return Vec3f(state_.vec_position(0), state_.vec_position(1), state_.angle);
}

ErrorType Vehicle::Ret3DofStateAtGeometryCenter(Vec3f *state) const {
  // 沿车辆纵向轴前移 d_cr，将后轴中心坐标转换为车身几何中心坐标。
  decimal_t cos_theta = cos(state_.angle);
  decimal_t sin_theta = sin(state_.angle);
  decimal_t x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  decimal_t y = state_.vec_position(1) + param_.d_cr() * sin_theta;
  (*state)(0) = x;
  (*state)(1) = y;
  (*state)(2) = state_.angle;
  return kSuccess;
}

void Vehicle::print() const {
  // Vehicle 自身输出标识，参数和运动状态交给各自对象输出。
  printf("\nVehicle:\n");
  printf(" -- ID:\t%d\n", id_);
  printf(" -- Subclass:\t%s\n", subclass_.c_str());
  param_.print();
  state_.print();
}

OrientedBoundingBox2D Vehicle::RetOrientedBoundingBox() const {
  // OBB 必须以几何中心为中心，而不是直接使用 State 的后轴中心位置。
  OrientedBoundingBox2D obb;
  double cos_theta = cos(state_.angle);
  double sin_theta = sin(state_.angle);
  obb.x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  obb.y = state_.vec_position(1) + param_.d_cr() * sin_theta;
  obb.angle = state_.angle;
  obb.width = param_.width();
  obb.length = param_.length();
  return obb;
}

ErrorType Vehicle::RetVehicleVertices(vec_E<Vec2f> *vertices) const {
  // 统一委托给 SemanticsUtils，避免多处重复车身角点计算公式。
  SemanticsUtils::GetVehicleVertices(param_, state_, vertices);
  return kSuccess;
}

ErrorType Vehicle::RetBumperVertices(std::array<Vec2f, 2> *vertices) const {
  // 先求几何中心，再沿航向正负方向移动半车长得到前后中点。
  decimal_t cos_theta = cos(state_.angle);
  decimal_t sin_theta = sin(state_.angle);

  decimal_t c_x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  decimal_t c_y = state_.vec_position(1) + param_.d_cr() * sin_theta;

  decimal_t d_lx = param_.length() / 2.0 * cos_theta;
  decimal_t d_ly = param_.length() / 2.0 * sin_theta;

  (*vertices)[0] = Vec2f(c_x - d_lx, c_y - d_ly);
  (*vertices)[1] = Vec2f(c_x + d_lx, c_y + d_ly);

  return kSuccess;
}

void VehicleSet::print() const {
  // unordered_map 不保证输出顺序，本函数仅用于人工调试而非确定性日志比较。
  printf("Vehicle Set Info:\n");
  for (auto iter = vehicles.begin(); iter != vehicles.end(); ++iter) {
    printf("\n -- ID. %d:\n", iter->first);
    iter->second.print();
  }
  printf("\n");
}

// 默认构造保持 is_openloop=false，因此 acc/steer_rate 为有效控制表示。
VehicleControlSignal::VehicleControlSignal() {}

VehicleControlSignal::VehicleControlSignal(double acc, double steer_rate)
    : acc(acc), steer_rate(steer_rate), is_openloop(false) {}

// 期望状态构造函数显式切换为开环模式，并将闭环控制量清零。
VehicleControlSignal::VehicleControlSignal(common::State state)
    : acc(0.0), steer_rate(0.0), is_openloop(true), state(state) {}

// 空元数据用于延迟加载地图尺寸与分辨率。
GridMapMetaInfo::GridMapMetaInfo() {}

GridMapMetaInfo::GridMapMetaInfo(const int w, const int h, const double res)
    : width(w), height(h), resolution(res) {
  // 物理覆盖范围等于栅格数量乘以单格分辨率。
  w_metric = w * resolution;
  h_metric = h * resolution;
}

void GridMapMetaInfo::print() const {
  // 同时输出离散尺寸与物理尺寸，便于检查分辨率配置。
  printf("GridMapMetaInfo:\n");
  printf(" -- width:%d\n", width);
  printf(" -- height:%d\n", height);
  printf(" -- resolution:%lf\n", resolution);
  printf(" -- w_metric:%lf\n", w_metric);
  printf(" -- h_metric:%lf\n", h_metric);
  printf("\n");
}

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND() {}

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(
    const std::array<int, N_DIM> &dims_size,
    const std::array<decimal_t, N_DIM> &dims_resolution,
    const std::array<std::string, N_DIM> &dims_name) {
  // 先保存结构元数据，再计算线性步长和元素总数，最后分配零填充数组。
  dims_size_ = dims_size;
  dims_resolution_ = dims_resolution;
  dims_name_ = dims_name;

  SetNDimSteps(dims_size_);
  SetDataSize(dims_size_);
  data_ = std::vector<T>(data_size_, 0);
  origin_.fill(0);
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, T *val) const {
  // 只有离散坐标读取接口会显式向调用方传播越界状态。
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return kWrongStatus;
  }
  // 有效 N 维坐标通过预计算步长展开为底层连续数组下标。
  int idx = GetMonoIdxUsingNDimIdx(coord);
  *val = data_[idx];
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, T *val) const {
  // 当前包装接口没有转发 GetValueUsingCoordinate 的错误码，这是既有 API 语义。
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  GetValueUsingCoordinate(coord, val);
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val_in, bool *res) const {
  // 世界位置先离散化；越界通过 res=false 表达，而不是错误码。
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val_in, bool *res) const {
  // 保持与全局位置版本相同的“错误转布尔值”语义。
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val) {
  // 写入前检查范围，防止非法下标访问底层数组。
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return kWrongStatus;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  data_[idx] = val;
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val) {
  // 当前包装接口与读取版本一致，不转发内部写入的越界错误码。
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  SetValueUsingCoordinate(coord, val);
  return kSuccess;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetCoordUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
  // round 将位置映射到最近栅格坐标，而不是包含该位置的 floor 栅格。
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  return coord;
}

template <typename T, int N_DIM>
std::array<decimal_t, N_DIM>
GridMapND<T, N_DIM>::GetRoundedPosUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
  // 先吸附到离散坐标，再按 origin + index*resolution 恢复世界位置。
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  std::array<decimal_t, N_DIM> round_pos = {};
  for (int i = 0; i < N_DIM; ++i) {
    round_pos[i] = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return round_pos;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalPositionUsingCoordinate(
    const std::array<int, N_DIM> &coord,
    std::array<decimal_t, N_DIM> *p_w) const {
  // 逐维执行线性坐标变换；当前不校验输入坐标是否位于地图范围内。
  auto ptr = p_w->data();
  for (int i = 0; i < N_DIM; ++i) {
    *(ptr + i) = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetCoordUsingGlobalMetricOnSingleDim(
    const decimal_t &metric, const int &i, int *idx) const {
  // 单维转换与 N 维版本保持相同的 round 规则。
  *idx = std::round((metric - origin_[i]) / dims_resolution_[i]);
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalMetricUsingCoordOnSingleDim(
    const int &idx, const int &i, decimal_t *metric) const {
  // 单维世界坐标由原点、下标和分辨率线性计算。
  *metric = idx * dims_resolution_[i] + origin_[i];
  return kSuccess;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRange(
    const std::array<int, N_DIM> &coord) const {
  // N 维坐标只有在每一维都位于半开区间内时才有效。
  for (int i = 0; i < N_DIM; ++i) {
    if (coord[i] < 0 || coord[i] >= dims_size_[i]) {
      return false;
    }
  }
  return true;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRangeOnSingleDim(const int &idx,
                                                       const int &i) const {
  // 调用方负责保证维度 i 合法，本函数只检查该维的坐标值。
  return (idx >= 0) && (idx < dims_size_[i]);
}

template <typename T, int N_DIM>
int GridMapND<T, N_DIM>::GetMonoIdxUsingNDimIdx(
    const std::array<int, N_DIM> &idx) const {
  // 第 0 维连续存储，其他维通过 dims_step_ 累加偏移。
  int mono_idx = 0;
  for (int i = 0; i < N_DIM; ++i) {
    mono_idx += dims_step_[i] * idx[i];
  }
  return mono_idx;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetNDimIdxUsingMonoIdx(
    const int &idx) const {
  // 从最高维开始做整除和取余，逐步还原各维坐标。
  std::array<int, N_DIM> idx_nd = {};
  int tmp = idx;
  for (int i = N_DIM - 1; i >= 0; --i) {
    idx_nd[i] = tmp / dims_step_[i];
    tmp = tmp % dims_step_[i];
  }
  return idx_nd;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetNDimSteps(
    const std::array<int, N_DIM> &dims_size) {
  // 步长为之前所有维度尺寸的累乘，第 0 维步长固定为 1。
  int step = 1;
  for (int i = 0; i < N_DIM; ++i) {
    dims_step_[i] = step;
    step = step * dims_size[i];
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetDataSize(
    const std::array<int, N_DIM> &dims_size) {
  // 理论元素数为各维尺寸乘积；当前实现实际读取成员 dims_size_。
  int total_ele_num = 1;
  for (int i = 0; i < N_DIM; ++i) {
    total_ele_num = total_ele_num * dims_size_[i];
  }
  data_size_ = total_ele_num;
  return kSuccess;
}

// 显式实例化项目中使用的 2D/3D uint8 和 int 栅格类型。
template class GridMapND<uint8_t, 2>;
template class GridMapND<uint8_t, 3>;
template class GridMapND<int, 2>;
template class GridMapND<int, 3>;

void LaneRaw::print() const {
  // 原始车道输出覆盖纵横向拓扑、行为属性和中心线概要。
  printf("Lane %d:\n", id);
  printf(" -- dir:\t%d\n", dir);

  printf(" -- child_id:\t[");
  for (const auto &id : child_id) {
    printf(" %d ", id);
  }
  printf("]\n");

  printf(" -- father_id:\t[");
  for (const auto &id : father_id) {
    printf(" %d ", id);
  }
  printf("]\n");
  printf(" -- length:\t%f\n", length);
  printf(" -- l_lane_id:\t%d\n", l_lane_id);
  printf(" -- l_change_avbl:\t%d\n", l_change_avbl);
  printf(" -- r_lane_id:\t%d\n", r_lane_id);
  printf(" -- r_change_avbl:\t%d\n", r_change_avbl);

  printf(" -- behavior:\t%s\n", behavior.c_str());

  printf(" -- start_point: (%f, %f)\n", start_point(0), start_point(1));
  printf(" -- final_point: (%f, %f)\n", final_point(0), final_point(1));
  printf(" -- point number:\t%d\n", static_cast<int>(lane_points.size()));
}

void LaneNet::print() const {
  // unordered_map 遍历顺序不稳定，本输出只用于人工检查地图内容。
  printf("LaneNet:\n");
  printf(" -- Number of lanes:\t%d\n", static_cast<int>(lane_set.size()));
  for (auto it = lane_set.begin(); it != lane_set.end(); ++it) {
    it->second.print();
  }
  printf("\n");
}

void SemanticLaneSet::print() const {
  // 语义 Lane 的几何细节由 Lane 自身接口查询，此处只输出集合规模。
  printf("SemanticLaneSet:\n");
  printf(" -- Number of lanes:\t%d\n", static_cast<int>(semantic_lanes.size()));
}

void CircleObstacle::print() const {
  // 类型码未在当前调试输出中展示，仅打印 ID 与圆形几何。
  printf("id: %d\n", id);
  circle.print();
  printf("\n");
}

void PolygonObstacle::print() const {
  // 类型码未在当前调试输出中展示，仅打印 ID 与多边形几何。
  printf("id: %d\n", id);
  polygon.print();
  printf("\n");
}

void ObstacleSet::print() const {
  // 两种障碍物分别存储，输出时依次遍历圆形和多边形集合。
  for (const auto &obs : obs_circle) {
    obs.second.print();
  }
  for (const auto &obs : obs_polygon) {
    obs.second.print();
  }
}

TrafficSignal::TrafficSignal()
    : start_point_(Vec2f::Zero()),
      end_point_(Vec2f::Zero()),
      valid_time_(Vec2f(0.0, std::numeric_limits<decimal_t>::max())),
      vel_range_(Vec2f::Zero()),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

TrafficSignal::TrafficSignal(const Vec2f &start_point, const Vec2f &end_point,
                             const Vec2f &valid_time, const Vec2f &vel_range)
    : start_point_(start_point),
      end_point_(end_point),
      valid_time_(valid_time),
      vel_range_(vel_range),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

void TrafficSignal::set_start_point(const Vec2f &start_point) {
  start_point_ = start_point;
}

void TrafficSignal::set_end_point(const Vec2f &end_point) {
  end_point_ = end_point;
}

void TrafficSignal::set_valid_time_til(const decimal_t max_valid_time) {
  valid_time_(1) = max_valid_time;
}

void TrafficSignal::set_valid_time_begin(const decimal_t min_valid_time) {
  valid_time_(0) = min_valid_time;
}

void TrafficSignal::set_valid_time(const Vec2f &valid_time) {
  valid_time_ = valid_time;
}

void TrafficSignal::set_vel_range(const Vec2f &vel_range) {
  vel_range_ = vel_range;
}

void TrafficSignal::set_lateral_range(const Vec2f &lateral_range) {
  lateral_range_ = lateral_range;
}

void TrafficSignal::set_max_velocity(const decimal_t max_velocity) {
  vel_range_(1) = max_velocity;
}

Vec2f TrafficSignal::start_point() const { return start_point_; }
Vec2f TrafficSignal::end_point() const { return end_point_; }
Vec2f TrafficSignal::valid_time() const { return valid_time_; }
Vec2f TrafficSignal::vel_range() const { return vel_range_; }
Vec2f TrafficSignal::lateral_range() const { return lateral_range_; }
decimal_t TrafficSignal::max_velocity() const { return vel_range_(1); }

SpeedLimit::SpeedLimit(const Vec2f &start_point, const Vec2f &end_point,
                       const Vec2f &vel_range)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    vel_range) {}

StoppingSign::StoppingSign(const Vec2f &start_point, const Vec2f &end_point)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    Vec2f::Zero()) {}

void TrafficLight::set_type(const Type &type) { type_ = type; }

TrafficLight::Type TrafficLight::type() const { return type_; }

ErrorType SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
    const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb) {
  double cos_theta = cos(s.angle);
  double sin_theta = sin(s.angle);
  obb->x = s.vec_position[0] + param.d_cr() * cos_theta;
  obb->y = s.vec_position[1] + param.d_cr() * sin_theta;
  obb->angle = s.angle;
  obb->length = param.length();
  obb->width = param.width();
  return kSuccess;
}

ErrorType SemanticsUtils::GetVehicleVertices(const VehicleParam &param,
                                             const State &state,
                                             vec_E<Vec2f> *vertices) {
  decimal_t angle = state.angle;

  decimal_t cos_theta = cos(angle);
  decimal_t sin_theta = sin(angle);

  decimal_t c_x = state.vec_position(0) + param.d_cr() * cos_theta;
  decimal_t c_y = state.vec_position(1) + param.d_cr() * sin_theta;

  decimal_t d_wx = param.width() / 2 * sin_theta;
  decimal_t d_wy = param.width() / 2 * cos_theta;
  decimal_t d_lx = param.length() / 2 * cos_theta;
  decimal_t d_ly = param.length() / 2 * sin_theta;

  // Counterclockwise from left-front vertex
  vertices->push_back(Vec2f(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
  // vertices->push_back(Vec2f(c_x - d_wx, c_y + d_wy));
  vertices->push_back(Vec2f(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
  // vertices->push_back(Vec2f(c_x - d_lx, c_y - d_ly));
  vertices->push_back(Vec2f(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
  // vertices->push_back(Vec2f(c_x + d_wx, c_y - d_wy));
  vertices->push_back(Vec2f(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
  // vertices->push_back(Vec2f(c_x + d_lx, c_y + d_ly));

  return kSuccess;
}

ErrorType SemanticsUtils::InflateVehicleBySize(const Vehicle &vehicle_in,
                                               const decimal_t delta_w,
                                               const decimal_t delta_l,
                                               Vehicle *vehicle_out) {
  common::Vehicle inflated_vehicle = vehicle_in;
  common::VehicleParam vehicle_param = vehicle_in.param();
  vehicle_param.set_width(vehicle_param.width() + delta_w);
  vehicle_param.set_length(vehicle_param.length() + delta_l);
  inflated_vehicle.set_param(vehicle_param);
  *vehicle_out = inflated_vehicle;
  return kSuccess;
}

}  // namespace common
