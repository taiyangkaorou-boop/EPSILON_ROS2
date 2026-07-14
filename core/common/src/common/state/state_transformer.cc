/**
 * @file state_transformer.cc
 * @brief 实现基于车道投影、切向和曲率的世界/Frenet 状态转换公式。
 */
#include "common/state/state_transformer.h"

namespace common {

ErrorType StateTransformer::GetStateFromFrenetState(const FrenetState& fs,
                                                     State* s) const {
  // 参考 Lane 和横向弧长导数必须有效，当前转换只支持二维平面车道。
  if (!lane_.IsValid()) {
    printf("[StateFromFrenetState]Err: lane not valid.\n");
    return kIllegalInput;
  }
  if (!fs.is_ds_usable) {
    // ~ You may come from high speed traj but vs = 0.
    return kIllegalInput;
  }

  if (LaneDim != 2) {
    printf("[StateFromFrenetState]Err: cannot support non-plane now.\n");
    return kIllegalInput;
  }

  // 在纵向弧长 s 处查询车道曲率、位置和单位切向。
  decimal_t curvature, curvature_derivative;
  if (lane_.GetCurvatureByArcLength(fs.vec_s[0], &curvature,
                                    &curvature_derivative) != kSuccess) {
    return kWrongStatus;
  }

  // 1-kappa*d 是 Frenet 坐标变换雅可比的关键分母，非正时映射退化。
  decimal_t one_minus_curd = 1 - curvature * fs.vec_ds[0];
  if (one_minus_curd < kEPS) {
    // ~ the violation is typically caused by lateral dependent trajectories
    // ~ with very small s (overshotting)
    return kWrongStatus;
  }

  Vecf<LaneDim> lane_pos;
  if (lane_.GetPositionByArcLength(fs.vec_s[0], &lane_pos) != kSuccess) {
    return kWrongStatus;
  }

  Vecf<LaneDim> vec_tangent;
  if (lane_.GetTangentVectorByArcLength(fs.vec_s[0], &vec_tangent) !=
      kSuccess) {
    return kWrongStatus;
  }

  decimal_t lane_orientation = vec2d_to_angle(vec_tangent);

  // 由左法向和横向偏移恢复世界位置，并由 d'(s) 恢复航向差。
  Vecf<LaneDim> vec_normal(-vec_tangent[1], vec_tangent[0]);
  decimal_t tan_delta_theta = fs.vec_ds[1] / one_minus_curd;
  decimal_t delta_theta = atan2(fs.vec_ds[1], one_minus_curd);
  decimal_t cn_delta_theta = cos(delta_theta);

  s->vec_position = vec_normal * fs.vec_ds[0] + lane_pos;
  s->velocity = fs.vec_s[1] * one_minus_curd / cn_delta_theta;
  s->angle = normalize_angle(delta_theta + lane_orientation);

  // 使用 Frenet 微分几何关系恢复世界曲率和切向加速度。
  decimal_t lhs = (fs.vec_ds[2] + (curvature_derivative * fs.vec_ds[0] +
                                   curvature * fs.vec_ds[1]) *
                                      tan_delta_theta) *
                  cn_delta_theta * cn_delta_theta / one_minus_curd;
  s->curvature = (lhs + curvature) * cn_delta_theta / one_minus_curd;
  decimal_t delta_theta_derivative = 1.0 /
                                     (1 + tan_delta_theta * tan_delta_theta) *
                                     (fs.vec_ds[2] * one_minus_curd +
                                      curvature * fs.vec_ds[1] * fs.vec_ds[1]) /
                                     pow(one_minus_curd, 2);
  s->acceleration =
      fs.vec_s[2] * one_minus_curd / cn_delta_theta +
      fs.vec_s[1] * fs.vec_s[1] / cn_delta_theta *
          (one_minus_curd * tan_delta_theta * delta_theta_derivative -
           (curvature_derivative * fs.vec_ds[0] + curvature * fs.vec_ds[1]));
  s->time_stamp = fs.time_stamp;  // pass the time stamp
  return kSuccess;
}

ErrorType StateTransformer::GetFrenetStateVectorFromStates(
    const vec_E<State> state_vec, vec_E<FrenetState>* fs_vec) const {
  // 输出先清空；逐元素转换，失败时不会回滚已经写入的前缀。
  fs_vec->clear();
  fs_vec->reserve(state_vec.size());
  FrenetState fs;
  for (const auto& state : state_vec) {
    if (GetFrenetStateFromState(state, &fs) == kSuccess) {
      fs_vec->push_back(fs);
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType StateTransformer::GetStateVectorFromFrenetStates(
    const vec_E<FrenetState>& fs_vec, vec_E<State>* state_vec) const {
  // 与反向批量转换保持相同的 fail-fast 和部分结果语义。
  State s;
  state_vec->clear();
  state_vec->reserve(fs_vec.size());
  for (auto& fs : fs_vec) {
    if (GetStateFromFrenetState(fs, &s) == kSuccess) {
      state_vec->push_back(s);
    } else {
      // TODO (@denny.ding): confirm this logic is correct
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType StateTransformer::GetFrenetStateFromState(const State& s,
                                                     FrenetState* fs) const {
  // 首先把世界位置投影到参考 Lane 的最近弧长位置。
  if (!lane_.IsValid()) {
    return kIllegalInput;
  }

  decimal_t arc_length;
  if (lane_.GetArcLengthByVecPosition(s.vec_position, &arc_length) !=
      kSuccess) {
    return kWrongStatus;
  }

  // 获取投影位置处的几何量，用于后续状态导数变换。
  decimal_t curvature, curvature_derivative;
  if (lane_.GetCurvatureByArcLength(arc_length, &curvature,
                                    &curvature_derivative) != kSuccess) {
    return kWrongStatus;
  }

  Vecf<2> lane_position;
  if (lane_.GetPositionByArcLength(arc_length, &lane_position) != kSuccess) {
    return kWrongStatus;
  }

  Vecf<2> lane_tangent_vec;
  if (lane_.GetTangentVectorByArcLength(arc_length, &lane_tangent_vec) !=
      kSuccess) {
    return kWrongStatus;
  }
  decimal_t lane_orientation = vec2d_to_angle(lane_tangent_vec);
  Vecf<2> lane_normal_vec = Vecf<2>(-lane_tangent_vec[1], lane_tangent_vec[0]);

  // 有限采样投影若在切向仍偏离 0.5 m 以上，则拒绝该转换结果。
  const decimal_t step_tolerance = 0.5;
  if (fabs((s.vec_position - lane_position).dot(lane_tangent_vec)) >
      step_tolerance) {
    // ~ projection deviates
    // ~ @(denny.ding) check whether this condition is good?
    return kWrongStatus;
  }

  // if (fabs(normalize_angle(s.angle - lane_orientation)) > M_PI / 2.0) {
  //   // printf("[FrenetStateFromState]State angle %lf not on lane (%lf).\n",
  //   //        s.angle, lane_orientation);
  //   return kWrongStatus;
  // }

  // 横向偏移为位置差在左法向上的投影。
  decimal_t d = (s.vec_position - lane_position).dot(lane_normal_vec);
  decimal_t one_minus_curd = 1 - curvature * d;
  if (one_minus_curd < kEPS) {
    // printf("[StateFromFrenetState]d not valid for transform.\n");
    return kWrongStatus;
  }
  // 航向差及其正切用于计算 d'(s)、d''(s) 和纵向 s 导数。
  decimal_t delta_theta = normalize_angle(s.angle - lane_orientation);

  decimal_t cn_delta_theta = cos(delta_theta);
  decimal_t tan_delta_theta = tan(delta_theta);
  decimal_t ds = one_minus_curd * tan_delta_theta;
  decimal_t dss =
      -(curvature_derivative * d + curvature * ds) * tan_delta_theta +
      one_minus_curd / pow(cn_delta_theta, 2) *
          (s.curvature * one_minus_curd / cn_delta_theta - curvature);
  decimal_t sp = s.velocity * cn_delta_theta / one_minus_curd;

  decimal_t delta_theta_derivative =
      1 / (1 + pow(tan_delta_theta, 2)) *
      (dss * one_minus_curd + curvature * pow(ds, 2)) / pow(one_minus_curd, 2);

  decimal_t spp =
      (s.acceleration -
       pow(sp, 2) / cn_delta_theta *
           (one_minus_curd * tan_delta_theta * delta_theta_derivative -
            (curvature_derivative * d + curvature * ds))) *
      cn_delta_theta / one_minus_curd;

  // 已求得 d 关于 s 的导数，因此使用 kInitWithDs 同步生成时间导数表示。
  fs->Load(Vecf<3>(arc_length, sp, spp), Vecf<3>(d, ds, dss),
           FrenetState::kInitWithDs);
  fs->time_stamp = s.time_stamp;
  return kSuccess;
}

ErrorType StateTransformer::GetFrenetPointFromPoint(const Vec2f& s,
                                                     Vec2f* fs) const {
  // 点转换只计算几何 `[s,d]`，不涉及速度、加速度或航向。
  if (!lane_.IsValid()) return kIllegalInput;
  decimal_t arc_length;
  if (lane_.GetArcLengthByVecPosition(s, &arc_length) != kSuccess) {
    return kWrongStatus;
  }

  Vecf<2> lane_position;
  if (lane_.GetPositionByArcLength(arc_length, &lane_position) != kSuccess) {
    return kWrongStatus;
  }

  Vecf<2> lane_tangent_vec;
  if (lane_.GetTangentVectorByArcLength(arc_length, &lane_tangent_vec) !=
      kSuccess) {
    return kWrongStatus;
  }
  Vecf<2> lane_normal_vec = Vecf<2>(-lane_tangent_vec[1], lane_tangent_vec[0]);

  decimal_t d = (s - lane_position).dot(lane_normal_vec);

  (*fs)(0) = arc_length;
  (*fs)(1) = d;

  return kSuccess;
}

ErrorType StateTransformer::GetFrenetPointVectorFromPoints(
    const vec_E<Vec2f>& pts, vec_E<Vec2f>* fps) const {
  // 输出先清空，逐点转换；失败时保留已成功转换的前缀。
  fps->clear();
  fps->reserve(pts.size());
  Vec2f fp;
  for (const auto& pt : pts) {
    if (GetFrenetPointFromPoint(pt, &fp) == kSuccess) {
      fps->push_back(fp);
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

}  // namespace common
