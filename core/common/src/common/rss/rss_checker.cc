#include "common/rss/rss_checker.h"

namespace common {

// 根据对象在前/后方以及双方速度方向，比较响应后制停距离得到纵向安全净间距。
ErrorType RssChecker::CalculateSafeLongitudinalDistance(
    const decimal_t ego_vel, const decimal_t other_vel,
    const LongitudinalDirection& direction, const RssConfig& config,
    decimal_t* distance) {
  decimal_t ret = 0.0;
  // 距离公式使用速度幅值，符号只用于区分同向、对向和未支持的倒车组合。
  decimal_t ego_vel_abs = fabs(ego_vel);
  decimal_t other_vel_abs = fabs(other_vel);
  decimal_t ego_vel_at_response_time =
      ego_vel_abs + config.longitudinal_acc_max * config.response_time;
  decimal_t other_vel_at_response_time =
      other_vel_abs + config.longitudinal_acc_max * config.response_time;

  decimal_t ego_distance_driven, other_distance_driven;
  if (direction == Front) {
    // 前方有对象时，自车在响应阶段继续加速，随后按最小制动能力停车。
    ego_distance_driven =
        (ego_vel_abs + ego_vel_at_response_time) / 2.0 * config.response_time +
        ego_vel_at_response_time * ego_vel_at_response_time /
            (2 * config.longitudinal_brake_min);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // 同向前车立即按最大制动能力停车，安全距离取两者制停路程差。
      other_distance_driven =
          (other_vel_abs * other_vel_abs) / (2 * config.longitudinal_brake_max);
      ret = ego_distance_driven - other_distance_driven;
    } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
      // 对向前车也在响应阶段继续接近，双方制停路程相加。
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      ret = ego_distance_driven + other_distance_driven;
    } else {
      // 自车倒车组合未建模，baseline 退化为零安全距离并仍报告成功。
      ret = 0.0;
    }
  } else if (direction == Rear) {
    // 后方有对象时，自车被视为立即最大制动，后车响应后再以最小制动停车。
    ego_distance_driven =
        ego_vel_abs * ego_vel_abs / (2 * config.longitudinal_brake_max);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // 同向后车的被动制停路程减去自车主动制停路程。
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      ret = other_distance_driven - ego_distance_driven;
    } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
      // 后方对象反向远离时不要求额外纵向距离。
      ret = 0.0;
    } else {
      // 自车倒车组合未建模。
      ret = 0.0;
    }
  }
  // 制停路程差为负时按零截断。
  *distance = ret > 0.0 ? ret : 0.0;
  return kSuccess;
}

// 将纵向安全距离不等式反解为自车允许的速度上下界。
ErrorType RssChecker::CalculateSafeLongitudinalVelocity(
    const decimal_t other_vel, const LongitudinalDirection& direction,
    const decimal_t& lon_distance_abs, const RssConfig& config,
    decimal_t* ego_vel_low, decimal_t* ego_vel_upp) {
  decimal_t other_vel_abs = fabs(other_vel);
  decimal_t other_vel_at_response_time =
      other_vel_abs + config.longitudinal_acc_max * config.response_time;
  decimal_t other_distance_driven;
  if (direction == Front) {
    if (other_vel >= 0.0) {
      // 同向前车立即最大制动；自车安全条件给出速度上界的二次方程。
      other_distance_driven =
          (other_vel_abs * other_vel_abs) / (2 * config.longitudinal_brake_max);
      // 取非负方向对应的二次方程根作为上界，baseline 不检查判别式。
      decimal_t a = 1.0 / (2.0 * config.longitudinal_brake_min);
      decimal_t b = config.response_time +
                    (config.longitudinal_acc_max * config.response_time /
                     config.longitudinal_brake_min);
      decimal_t c = 0.5 *
                        (config.longitudinal_acc_max +
                         pow(config.longitudinal_acc_max, 2) /
                             config.longitudinal_brake_min) *
                        pow(config.response_time, 2) -
                    other_distance_driven - lon_distance_abs;
      *ego_vel_upp = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
      *ego_vel_low = 0.0;
    } else {
      // 对向对象先占用自身响应与制停距离；距离不足时自车上界直接置零。
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      if (other_distance_driven > lon_distance_abs) {
        *ego_vel_upp = 0.0;
        *ego_vel_low = 0.0;
      } else {
        // 剩余净距离用于反解自车速度上界。
        decimal_t a = 1.0 / (2.0 * config.longitudinal_brake_min);
        decimal_t b = config.response_time +
                      (config.longitudinal_acc_max * config.response_time /
                       config.longitudinal_brake_min);
        decimal_t c = 0.5 *
                          (config.longitudinal_acc_max +
                           pow(config.longitudinal_acc_max, 2) /
                               config.longitudinal_brake_min) *
                          pow(config.response_time, 2) -
                      (lon_distance_abs - other_distance_driven);
        *ego_vel_upp = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
        *ego_vel_low = 0.0;
      }
    }
  } else {
    if (other_vel >= 0.0) {
      // 同向后车形成“自车不能过慢”的安全下界，上界保持无穷。
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      if (other_distance_driven < lon_distance_abs) {
        *ego_vel_upp = kInf;
        *ego_vel_low = 0.0;
      } else {
        *ego_vel_upp = kInf;
        *ego_vel_low = sqrt(2 * config.longitudinal_brake_max *
                            (other_distance_driven - lon_distance_abs));
      }
    } else {
      // 后方对象反向远离时不限制自车速度。
      *ego_vel_upp = kInf;
      *ego_vel_low = 0.0;
    }
  }
  return kSuccess;
}

// 按左右相对位置和横向速度符号组合，比较主动/被动横向制停路程。
ErrorType RssChecker::CalculateSafeLateralDistance(
    const decimal_t ego_vel, const decimal_t other_vel,
    const LateralDirection& direction, const RssConfig& config,
    decimal_t* distance) {
  decimal_t ret = 0.0;
  decimal_t ego_lat_vel_abs = fabs(ego_vel);
  decimal_t other_lat_vel_abs = fabs(other_vel);
  // lateral_miu 在此不是摩擦系数，而是最终直接相加的固定距离裕量。
  decimal_t distance_correction = config.lateral_miu;
  decimal_t ego_lat_vel_at_response_time =
      ego_lat_vel_abs + config.response_time * config.lateral_acc_max;
  decimal_t other_lat_vel_at_response_time =
      other_lat_vel_abs + config.response_time * config.lateral_acc_max;
  decimal_t ego_active_brake_distance =
      ego_lat_vel_abs * ego_lat_vel_abs / (2 * config.lateral_brake_max);
  // 被动制动模型包含响应阶段加速和随后按最小制动能力停车。
  decimal_t ego_passive_brake_distance =
      (ego_lat_vel_abs + ego_lat_vel_at_response_time) / 2.0 *
          config.response_time +
      ego_lat_vel_at_response_time * ego_lat_vel_at_response_time /
          (2 * config.lateral_brake_min);
  decimal_t other_active_brake_distance =
      other_lat_vel_abs * other_lat_vel_abs / (2 * config.lateral_brake_max);
  decimal_t other_passive_brake_distance =
      (other_lat_vel_abs + other_lat_vel_at_response_time) / 2.0 *
          config.response_time +
      other_lat_vel_at_response_time * other_lat_vel_at_response_time /
          (2 * config.lateral_brake_min);
  if (direction == Right) {
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // 对方在右侧，双方都向左：对方被动制停路程减自车主动制停路程。
      ret = other_passive_brake_distance - ego_active_brake_distance;
    } else if (ego_vel >= 0.0 && other_vel < 0.0) {
      // 双方横向远离。
      ret = 0.0;
    } else if (ego_vel < 0.0 && other_vel < 0.0) {
      // 双方都向右：自车被动制停路程减对方主动制停路程。
      ret = ego_passive_brake_distance - other_active_brake_distance;
    } else if (ego_vel < 0.0 && other_vel >= 0.0) {
      // 双方相向横移，两个被动制停路程相加。
      ret = ego_passive_brake_distance + other_passive_brake_distance;
    } else {
      ret = 0.0;
    }
  } else if (direction == Left) {
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // 对方在左侧，双方都向左：自车被动制停路程减对方主动制停路程。
      ret = ego_passive_brake_distance - other_active_brake_distance;
    } else if (ego_vel >= 0.0 && other_vel < 0.0) {
      // 双方相向横移。
      ret = ego_passive_brake_distance + other_passive_brake_distance;
    } else if (ego_vel < 0.0 && other_vel < 0.0) {
      // 双方都向右：对方被动制停路程减自车主动制停路程。
      ret = other_passive_brake_distance - ego_active_brake_distance;
    } else if (ego_vel < 0.0 && other_vel >= 0.0) {
      // 双方横向远离。
      ret = 0.0;
    } else {
      ret = 0.0;
    }
  }
  // 运动学差值先截断为非负，再无条件附加固定横向裕量。
  ret = ret > 0.0 ? ret : 0.0;
  ret += distance_correction;
  *distance = ret;
  return kSuccess;
}

// 从两个至少含纵/横速度分量的动态数组生成固定顺序的二维安全距离输出。
ErrorType RssChecker::CalculateRssSafeDistances(
    const std::vector<decimal_t>& ego_vels,
    const std::vector<decimal_t>& other_vels,
    const LongitudinalDirection& lon_direct, const LateralDirection& lat_direct,
    const RssConfig& config, std::vector<decimal_t>* safe_distances) {
  // 输出采用覆盖语义，但 baseline 不验证两个输入向量长度至少为 2。
  safe_distances->clear();
  decimal_t safe_long_distance, safe_lat_distance;
  CalculateSafeLongitudinalDistance(ego_vels[0], other_vels[0], lon_direct,
                                    config, &safe_long_distance);
  CalculateSafeLateralDistance(ego_vels[1], other_vels[1], lat_direct, config,
                               &safe_lat_distance);
  safe_distances->push_back(safe_long_distance);
  safe_distances->push_back(safe_lat_distance);
  return kSuccess;
}

// 使用 Frenet 点质量位置与速度做二维“纵向危险且横向危险”的合取判定。
ErrorType RssChecker::RssCheck(const FrenetState& ego_fs,
                               const FrenetState& other_fs,
                               const RssConfig& config, bool* is_safe) {
  LongitudinalDirection lon_direct;
  LateralDirection lat_direct;
  // s 相等时按对方在后方处理，否则根据 s 大小确定前后关系。
  if (ego_fs.vec_s[0] >= other_fs.vec_s[0]) {
    lon_direct = Rear;
  } else {
    lon_direct = Front;
  }

  // d 相等时按对方在右侧处理。
  if (ego_fs.vec_dt[0] >= other_fs.vec_dt[0]) {
    lat_direct = Right;
  } else {
    lat_direct = Left;
  }

  std::vector<decimal_t> ego_vels{ego_fs.vec_s[1], ego_fs.vec_dt[1]};
  std::vector<decimal_t> other_vels{other_fs.vec_s[1], other_fs.vec_dt[1]};
  std::vector<decimal_t> safe_distances;
  CalculateRssSafeDistances(ego_vels, other_vels, lon_direct, lat_direct,
                            config, &safe_distances);

  // 两个方向必须同时侵入安全距离才判不安全；阈值相等时视为安全。
  if (fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) < safe_distances[0] &&
      fabs(ego_fs.vec_dt[0] - other_fs.vec_dt[0]) < safe_distances[1]) {
    *is_safe = false;
  } else {
    *is_safe = true;
  }
  return kSuccess;
}

// 面向真实车辆的检查：先投影到共同参考车道，再考虑车宽/车长并输出速度修正区间。
ErrorType RssChecker::RssCheck(const Vehicle& ego_vehicle,
                               const Vehicle& other_vehicle,
                               const StateTransformer& stf, const RssConfig& config,
                               bool* is_safe, LongitudinalViolateType* lon_type,
                               decimal_t* rss_vel_low, decimal_t* rss_vel_up) {
  FrenetState ego_fs, other_fs;
  // 转换器由调用方复用传入，避免每次检查重新构造并复制参考 Lane。
  if (stf.GetFrenetStateFromState(ego_vehicle.state(), &ego_fs) != kSuccess) {
    printf("[RssChecker]ego not on ref lane.\n");
    return kWrongStatus;
  }
  if (stf.GetFrenetStateFromState(other_vehicle.state(), &other_fs) !=
      kSuccess) {
    printf("[RssChecker]other %d not on ref lane.\n", other_vehicle.id());
    return kWrongStatus;
  }

  LongitudinalDirection lon_direct;
  LateralDirection lat_direct;

  if (ego_fs.vec_s[0] >= other_fs.vec_s[0]) {
    lon_direct = Rear;
  } else {
    lon_direct = Front;
  }

  if (ego_fs.vec_dt[0] >= other_fs.vec_dt[0]) {
    lat_direct = Right;
  } else {
    lat_direct = Left;
  }

  // 自车倒车未建模，baseline 直接判安全并返回零速度区间。
  if (ego_fs.vec_s[1] < 0.0) {
    *is_safe = true;
    *lon_type = LongitudinalViolateType::Legal;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
    return kSuccess;
  }

  decimal_t safe_lat_distance;
  CalculateSafeLateralDistance(ego_fs.vec_dt[1], other_fs.vec_dt[1], lat_direct,
                               config, &safe_lat_distance);
  // 点质量横向安全距离再加两车半宽，把 d 中心间距转换为车身净空要求。
  safe_lat_distance +=
      0.5 * (ego_vehicle.param().width() + other_vehicle.param().width());

  // 横向已充分分离时不再检查纵向关系。
  if (fabs(ego_fs.vec_dt[0] - other_fs.vec_dt[0]) > safe_lat_distance) {
    *is_safe = true;
    *lon_type = LongitudinalViolateType::Legal;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
    return kSuccess;
  }

  decimal_t lon_distance_abs, ego_vel_low, ego_vel_upp;
  if (lon_direct == Rear) {
    // 对方在后：从两后轴投影差中扣除对方前悬和自车后悬。
    decimal_t other_rear_wheel_to_front_bump =
        0.5 * other_vehicle.param().length() + other_vehicle.param().d_cr();
    decimal_t ego_rear_wheel_to_back_bump =
        fabs(0.5 * ego_vehicle.param().length() - ego_vehicle.param().d_cr());
    lon_distance_abs = fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) -
                       other_rear_wheel_to_front_bump -
                       ego_rear_wheel_to_back_bump;
  } else if (lon_direct == Front) {
    // 对方在前：扣除自车前悬和对方后悬，得到保险杠净间距。
    decimal_t ego_rear_wheel_to_front_bump =
        0.5 * ego_vehicle.param().length() + ego_vehicle.param().d_cr();
    decimal_t other_rear_wheel_to_back_bump = fabs(
        0.5 * other_vehicle.param().length() - other_vehicle.param().d_cr());
    lon_distance_abs = fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) -
                       ego_rear_wheel_to_front_bump -
                       other_rear_wheel_to_back_bump;
  }

  // 前向保险杠已重叠时立即判为“过快”，要求速度区间退化到零。
  if (lon_distance_abs < 0.0 && lon_direct == Front) {
    *is_safe = false;
    *lon_type = LongitudinalViolateType::TooFast;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
    return kSuccess;
  }

  CalculateSafeLongitudinalVelocity(other_fs.vec_s[1], lon_direct,
                                    lon_distance_abs, config, &ego_vel_low,
                                    &ego_vel_upp);
  // 当前速度高于前向上界为 TooFast，低于后向下界为 TooSlow。
  if (ego_fs.vec_s[1] > ego_vel_upp + kEPS) {
    *is_safe = false;
    *lon_type = LongitudinalViolateType::TooFast;
    *rss_vel_up = ego_vel_upp;
    *rss_vel_low = ego_vel_low;
  } else if (ego_fs.vec_s[1] < ego_vel_low - kEPS) {
    *is_safe = false;
    *lon_type = LongitudinalViolateType::TooSlow;
    *rss_vel_up = ego_vel_upp;
    *rss_vel_low = ego_vel_low;
  } else {
    *is_safe = true;
    *lon_type = LongitudinalViolateType::Legal;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
  }
  return kSuccess;
}

}  // namespace common
