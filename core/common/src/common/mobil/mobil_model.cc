#include "common/mobil/mobil_model.h"

namespace common {

// 计算自车留在当前车道和假设离开当前车道时，自车/原后车的 IDM 加速度。
ErrorType MobilLaneChangingModel::GetMobilAccChangesOnCurrentLane(
    const FrenetState &cur_fs, const Vehicle &leading_vehicle,
    const FrenetState &leading_fs, const Vehicle &following_vehicle,
    const FrenetState &following_fs, decimal_t *acc_o, decimal_t *acc_o_tilda,
    decimal_t *acc_c) {
  bool has_leading_vehicle =
      leading_vehicle.id() == kInvalidAgentId ? false : true;
  bool has_following_vehicle =
      following_vehicle.id() == kInvalidAgentId ? false : true;

  decimal_t acc_o_tmp = 0.0, acc_o_tilda_tmp = 0.0, acc_c_tmp = 0.0;

  // MOBIL 层把各车当前纵向速度设为 IDM 期望速度，因此自由流状态倾向零加速度。
  IntelligentDriverModel::Param idm_param_o;
  idm_param_o.kDesiredVelocity = following_fs.vec_s(1);
  IntelligentDriverModel::Param idm_param_c;
  idm_param_c.kDesiredVelocity = cur_fs.vec_s(1);

  if ((!has_following_vehicle) || fabs(following_fs.vec_s(1)) < kEPS) {
    // 没有后车或后车近似静止时，不计算原后车收益，只计算自车当前车道加速度。
    if (!has_leading_vehicle) {
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, common::FrenetState(),
                                     true, &acc_c_tmp);
    } else {
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, leading_fs, false,
                                     &acc_c_tmp);
    }
  } else {
    if (!has_leading_vehicle) {
      // 无真实前车时使用远端同速虚拟前车近似自由道路。
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, cur_fs, false,
                                     &acc_o_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs,
                                     common::FrenetState(), true,
                                     &acc_o_tilda_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, common::FrenetState(),
                                     true, &acc_c_tmp);
    } else {
      // 有真实前车时分别计算原后车面对自车/前车和自车面对前车的 ACC 加速度。
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, cur_fs, false,
                                     &acc_o_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, leading_fs,
                                     false, &acc_o_tilda_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, leading_fs, false,
                                     &acc_c_tmp);
    }
  }
  *acc_o = acc_o_tmp;
  *acc_o_tilda = acc_o_tilda_tmp;
  *acc_c = acc_c_tmp;
  return kSuccess;
}

// 先用简化 RSS 门控目标车道，再计算自车插入前后的 IDM 加速度变化。
ErrorType MobilLaneChangingModel::GetMobilAccChangesOnTargetLane(
    const FrenetState &projected_cur_fs, const Vehicle &leading_vehicle,
    const FrenetState &leading_fs, const Vehicle &following_vehicle,
    const FrenetState &following_fs, bool *is_lc_safe, decimal_t *acc_n,
    decimal_t *acc_n_tilda, decimal_t *acc_c_tilda) {
  bool has_leading_vehicle =
      leading_vehicle.id() == kInvalidAgentId ? false : true;
  bool has_following_vehicle =
      following_vehicle.id() == kInvalidAgentId ? false : true;

  decimal_t acc_n_tmp = 0.0, acc_n_tilda_tmp = 0.0, acc_c_tilda_tmp = 0.0;

  *is_lc_safe = false;
  // 同一有效车辆不能同时作为目标车道前车和后车，否则邻车关联无效。
  if (leading_vehicle.id() == following_vehicle.id() &&
      leading_vehicle.id() != kInvalidAgentId) {
    *is_lc_safe = false;
  } else {
    // baseline 无论邻车 ID 是否有效都对传入 FrenetState 执行点质量 RSS 检查。
    bool is_front_safe = true, is_rear_safe = true;
    RssChecker::RssCheck(projected_cur_fs, leading_fs,
                         common::RssChecker::RssConfig(), &is_front_safe);
    RssChecker::RssCheck(projected_cur_fs, following_fs,
                         common::RssChecker::RssConfig(), &is_rear_safe);
    *is_lc_safe = is_front_safe && is_rear_safe;
  }

  // 只有安全门通过才写三个加速度输出；失败时调用方原值保持不变。
  if (*is_lc_safe) {
    IntelligentDriverModel::Param idm_param_n;
    idm_param_n.kDesiredVelocity = following_fs.vec_s(1);
    IntelligentDriverModel::Param idm_param_c;
    idm_param_c.kDesiredVelocity = projected_cur_fs.vec_s(1);

    if ((!has_following_vehicle) || fabs(following_fs.vec_s(1)) < kEPS) {
      // 目标后车缺失或近似静止时只计算自车换道后加速度。
      if (!has_leading_vehicle) {
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       common::FrenetState(), true,
                                       &acc_c_tilda_tmp);
      } else {
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       leading_fs, false, &acc_c_tilda_tmp);
      }
    } else {
      if (!has_leading_vehicle) {
        // 无目标前车时以同速远端虚拟前车表示自由道路。
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       common::FrenetState(), true, &acc_n_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       projected_cur_fs, false,
                                       &acc_n_tilda_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       common::FrenetState(), true,
                                       &acc_c_tilda_tmp);
      } else {
        // 有目标前车时计算目标后车插入前/后以及自车插入后的加速度。
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs, leading_fs,
                                       false, &acc_n_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       projected_cur_fs, false,
                                       &acc_n_tilda_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       leading_fs, false, &acc_c_tilda_tmp);
      }
    }
    *acc_n = acc_n_tmp;
    *acc_n_tilda = acc_n_tilda_tmp;
    *acc_c_tilda = acc_c_tilda_tmp;
  }
  return kSuccess;
}

// 在真实前车和同速远端虚拟前车两种场景下复用 ACC 纵向公式。
ErrorType MobilLaneChangingModel::GetDesiredAccelerationUsingIdm(
    const IntelligentDriverModel::Param &param, const FrenetState &rear_fs,
    const FrenetState &front_fs, const bool &use_virtual_front,
    decimal_t *acc) {
  IntelligentDriverModel::State idm_state;
  if (!use_virtual_front) {
    // 使用两车 s/v 直接构造一维跟驰状态，车辆长度沿用 IDM 默认固定值。
    idm_state =
        IntelligentDriverModel::State(rear_fs.vec_s(0), rear_fs.vec_s(1),
                                      front_fs.vec_s(0), front_fs.vec_s(1));
  } else {
    // 虚拟前车与后车同速，位置设为当前坐标外加 100 m 和 10 s 速度项。
    idm_state = IntelligentDriverModel::State(
        0.0, rear_fs.vec_s(1), 100.0 + rear_fs.vec_s(1) * 10, rear_fs.vec_s(1));
  }

  IntelligentDriverModel::GetAccDesiredAcceleration(param, idm_state, acc);
  return kSuccess;
}

}  // namespace common
