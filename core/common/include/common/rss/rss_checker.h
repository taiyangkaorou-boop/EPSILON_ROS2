#ifndef _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__
#define _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"

namespace common {

/**
 * @brief 基于简化 Responsibility-Sensitive Safety 运动学公式的无状态检查工具。
 *
 * 所有计算都在参考车道 Frenet 坐标中进行：s 为纵向、d 为横向。该类提供点质量安全
 * 距离、车辆尺寸修正后的安全速度区间，以及 MOBIL 使用的简化安全判定。
 */
class RssChecker {
 public:
  /// 另一交通参与者相对自车位于纵向前方或后方。
  enum LongitudinalDirection { Front = 0, Rear };
  /// 另一交通参与者相对自车位于横向左侧或右侧。
  enum LateralDirection { Left = 0, Right };
  /// 自车纵向速度相对 RSS 可行区间的分类。
  enum class LongitudinalViolateType { Legal = 0, TooFast, TooSlow };

  /**
   * @brief RSS 响应、加速和制动能力参数。
   *
   * 所有制动字段按正的减速度幅值解释，lateral_miu 在实现中作为固定横向距离裕量
   * 直接相加。结构体不验证参数非负或最大/最小制动关系。
   */
  struct RssConfig {
    /// 危险出现后允许继续响应的时间。
    decimal_t response_time = 0.1;
    /// 响应阶段纵向最大加速度。
    decimal_t longitudinal_acc_max = 2.0;
    /// 被动/舒适纵向最小制动减速度幅值。
    decimal_t longitudinal_brake_min = 4.0;
    /// 主动/紧急纵向最大制动减速度幅值。
    decimal_t longitudinal_brake_max = 5.0;
    /// 响应阶段横向最大加速度。
    decimal_t lateral_acc_max = 1.0;
    /// 被动横向最小制动减速度幅值。
    decimal_t lateral_brake_min = 1.0;
    /// 主动横向最大制动减速度幅值。
    decimal_t lateral_brake_max = 1.0;
    /// 无论相对横向速度如何都附加的最小横向距离裕量。
    decimal_t lateral_miu = 0.5;

    /// 使用默认 RSS 参数。
    RssConfig() {}
    /// 使用调用方给定的完整参数集，不执行合法性校验。
    RssConfig(const decimal_t _response_time,
              const decimal_t _longitudinal_acc_max,
              const decimal_t _longitudinal_brake_min,
              const decimal_t _longitudinal_brake_max,
              const decimal_t _lateral_acc_max,
              const decimal_t _lateral_brake_min,
              const decimal_t _lateral_brake_max, const decimal_t _lateral_miu)
        : response_time(_response_time),
          longitudinal_acc_max(_longitudinal_acc_max),
          longitudinal_brake_min(_longitudinal_brake_min),
          longitudinal_brake_max(_longitudinal_brake_max),
          lateral_acc_max(_lateral_acc_max),
          lateral_brake_min(_lateral_brake_min),
          lateral_brake_max(_lateral_brake_max),
          lateral_miu(_lateral_miu) {}
  };

  /**
   * @brief 计算点质量自车与前/后方对象之间的纵向安全净间距。
   * @param ego_vel 自车沿 s 的有符号速度。
   * @param other_vel 对方沿 s 的有符号速度。
   * @param direction 对方位于自车前方还是后方。
   * @param config RSS 参数。
   * @param distance 输出非负安全距离。
   */
  static ErrorType CalculateSafeLongitudinalDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LongitudinalDirection& direction, const RssConfig& config,
      decimal_t* distance);

  /**
   * @brief 计算点质量自车与左/右侧对象之间的横向安全净间距。
   * @param ego_vel 自车 d 对时间的有符号速度。
   * @param other_vel 对方 d 对时间的有符号速度。
   * @param direction 对方位于自车左侧还是右侧。
   * @param config RSS 参数。
   * @param distance 输出包含 lateral_miu 的非负安全距离。
   */
  static ErrorType CalculateSafeLateralDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LateralDirection& direction, const RssConfig& config,
      decimal_t* distance);

  /// 由 `[纵向速度, 横向速度]` 批量计算 `[纵向距离, 横向距离]`。
  static ErrorType CalculateRssSafeDistances(
      const std::vector<decimal_t>& ego_vels,
      const std::vector<decimal_t>& other_vels,
      const LongitudinalDirection& long_direct,
      const LateralDirection& lat_direct, const RssConfig& config,
      std::vector<decimal_t>* safe_distances);

  /**
   * @brief 对两个点质量 FrenetState 做二维 RSS 距离检查。
   * @note 只有纵向和横向距离同时小于对应安全距离时判为不安全，不包含车辆尺寸。
   */
  static ErrorType RssCheck(const FrenetState& ego_fs,
                            const FrenetState& other_fs,
                            const RssConfig& config, bool* is_safe);

  /**
   * @brief 将两车投影到同一参考车道，结合车身尺寸计算 RSS 纵向速度可行区间。
   * @param lon_type 输出合法、过快或过慢分类。
   * @param rss_vel_low 违规时输出安全速度下界；合法时实现写零。
   * @param rss_vel_up 违规时输出安全速度上界；合法时实现写零。
   */
  static ErrorType RssCheck(const Vehicle& ego_vehicle,
                            const Vehicle& other_vehicle, const StateTransformer& stf,
                            const RssConfig& config, bool* is_safe,
                            LongitudinalViolateType* lon_type,
                            decimal_t* rss_vel_low, decimal_t* rss_vel_up);

  /**
   * @brief 已知纵向净间距和对方速度，反解自车纵向安全速度区间。
   * @param ego_vel_low 输出下界；后方威胁可能产生正下界。
   * @param ego_vel_upp 输出上界；前方威胁限制上界。
   */
  static ErrorType CalculateSafeLongitudinalVelocity(
      const decimal_t other_vel, const LongitudinalDirection& direction,
      const decimal_t& lon_distance_abs, const RssConfig& config,
      decimal_t* ego_vel_low, decimal_t* ego_vel_upp);
};

}  // namespace common

#endif
