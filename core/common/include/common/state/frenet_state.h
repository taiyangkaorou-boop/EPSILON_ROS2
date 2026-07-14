/**
 * @file frenet_state.h
 * @brief 定义纵向 s 与横向 d 的时间/弧长导数联合 Frenet 状态。
 */
#ifndef _COMMON_INC_COMMON_STATE_FRENET_STATE_H__
#define _COMMON_INC_COMMON_STATE_FRENET_STATE_H__

namespace common {
/**
 * @brief 同时保存纵向 s 状态和横向 d 关于时间/弧长导数的 Frenet 状态。
 *
 * vec_s=`[s, ds/dt, d2s/dt2]`，vec_dt=`[d, dd/dt, d2d/dt2]`，
 * vec_ds=`[d, dd/ds, d2d/ds2]`。当纵向速度接近零时无法从时间导数稳定转换为
 * 弧长导数，is_ds_usable 会被置为 false。
 */
struct FrenetState {
  /// 指定 Load 的横向输入是时间导数还是弧长导数。
  enum InitType { kInitWithDt, kInitWithDs };
  decimal_t time_stamp{0.0};
  Vecf<3> vec_s{Vecf<3>::Zero()};
  Vecf<3> vec_dt{Vecf<3>::Zero()};
  Vecf<3> vec_ds{Vecf<3>::Zero()};
  bool is_ds_usable = true;

  /// 构造全零且默认认为 ds 表示可用的 Frenet 状态。
  FrenetState() {}

  /**
   * @brief 设置纵向状态，并在 dt/ds 两种横向导数表示之间完成链式法则转换。
   * @param s `[s, s_dot, s_ddot]`。
   * @param d 横向三元组，其导数含义由 type 指定。
   * @param type kInitWithDt 或 kInitWithDs；其他值触发断言。
   */
  void Load(const Vecf<3>& s, const Vecf<3>& d, const InitType& type) {
    vec_s = s;
    if (type == kInitWithDt) {
      vec_dt = d;
      vec_ds[0] = vec_dt[0];
      // s_dot 非零时可由时间导数反求对弧长的一、二阶导数。
      if (fabs(vec_s[1]) > kEPS) {
        vec_ds[1] = vec_dt[1] / vec_s[1];
        vec_ds[2] = (vec_dt[2] - vec_ds[1] * vec_s[2]) / (vec_s[1] * vec_s[1]);
        is_ds_usable = true;
      } else {
        vec_ds[1] = 0.0;
        vec_ds[2] = 0.0;
        is_ds_usable = false;
      }
    } else if (type == kInitWithDs) {
      // 已知弧长导数时，使用 d/dt=(d/ds)*s_dot 恢复时间导数。
      vec_ds = d;
      vec_dt[0] = vec_ds[0];
      vec_dt[1] = vec_s[1] * vec_ds[1];
      vec_dt[2] = vec_ds[2] * vec_s[1] * vec_s[1] + vec_ds[1] * vec_s[2];
      is_ds_usable = true;
    } else {
      assert(false);
    }
  }

  /// 直接使用已一致的 s、dt、ds 三元组构造，不进行交叉校验。
  FrenetState(const Vecf<3>& s, const Vecf<3>& dt, const Vecf<3>& ds) {
    vec_s = s;
    vec_dt = dt;
    vec_ds = ds;
  }

  /// 输出时间戳和三组 Frenet 分量，并提示 ds 表示是否可用。
  void print() const {
    printf("frenet state stamp: %lf.\n", time_stamp);
    printf("-- vec_s: (%lf, %lf, %lf).\n", vec_s[0], vec_s[1], vec_s[2]);
    printf("-- vec_dt: (%lf, %lf, %lf).\n", vec_dt[0], vec_dt[1], vec_dt[2]);
    printf("-- vec_ds: (%lf, %lf, %lf).\n", vec_ds[0], vec_ds[1], vec_ds[2]);
    if (!is_ds_usable) printf("-- warning: ds not usable.\n");
  }
};

}  // namespace common

#endif
