#ifndef _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__
#define _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/rss/rss_checker.h"

namespace common {

/**
 * @brief 基于 IDM 加速度变化和简化 RSS 门控的 MOBIL 换道量计算工具。
 *
 * 记 c 为自车、o 为原车道后车、n 为目标车道后车，tilda 表示假设自车完成换道后的
 * 加速度。该类只计算这些中间量，不在此应用礼让系数或换道收益阈值。
 */
class MobilLaneChangingModel {
 public:
  /**
   * @brief 计算当前车道下原后车换道前/后加速度及自车保持车道加速度。
   * @param acc_o 原车道后车在自车仍位于当前车道时的加速度。
   * @param acc_o_tilda 自车离开后，原车道后车面对原前车/虚拟前车的加速度。
   * @param acc_c 自车保持当前车道时的加速度。
   */
  static ErrorType GetMobilAccChangesOnCurrentLane(
      const FrenetState &cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, decimal_t *acc_o, decimal_t *acc_o_tilda,
      decimal_t *acc_c);

  /**
   * @brief 检查目标车道 RSS 安全性并计算目标后车与自车换道前/后加速度。
   * @param is_lc_safe 输出简化 RSS 前后方检查的合取结果。
   * @param acc_n 目标车道后车在自车插入前的加速度。
   * @param acc_n_tilda 自车插入后目标车道后车的加速度。
   * @param acc_c_tilda 自车位于目标车道后的加速度。
   * @note 判为不安全时当前实现不写三个加速度输出。
   */
  static ErrorType GetMobilAccChangesOnTargetLane(
      const FrenetState &projected_cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, bool *is_lc_safe, decimal_t *acc_n,
      decimal_t *acc_n_tilda, decimal_t *acc_c_tilda);

 private:
  /// 将两 FrenetState 封装为 IDM 状态；无真实前车时构造同速远端虚拟前车。
  static ErrorType GetDesiredAccelerationUsingIdm(
      const IntelligentDriverModel::Param &param, const FrenetState &rear_fs,
      const FrenetState &front_fs, const bool &use_virtual_front,
      decimal_t *acc);
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__
