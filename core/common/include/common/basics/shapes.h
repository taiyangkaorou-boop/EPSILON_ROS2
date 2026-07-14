/**
 * @file shapes.h
 * @author HKUST Aerial Robotics Group
 * @brief 定义规划系统使用的基础几何对象，以及 OBB/AABB 碰撞和坐标转换工具。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_SHAPES_H__
#define _COMMON_INC_COMMON_BASICS_SHAPES_H__

#include <map>
#include <opencv2/core/core.hpp>

#include "common/basics/basics.h"
#include "common/basics/colormap.h"
#include "common/math/calculations.h"

namespace common {

/// 三维浮点点；二维算法忽略 z 分量，但保留它用于消息和可视化接口。
struct Point {
  decimal_t x = 0.0;
  decimal_t y = 0.0;
  decimal_t z = 0.0;

  /**
   * @brief 构造位于原点的三维点。
   */
  Point();

  /**
   * @brief 构造二维点，z 保持默认值 0。
   *
   * @param _x 世界坐标 x。
   * @param _y 世界坐标 y。
   */
  Point(decimal_t _x, decimal_t _y);

  /**
   * @brief 使用完整三维坐标构造点。
   *
   * @param _x 世界坐标 x。
   * @param _y 世界坐标 y。
   * @param _z 世界坐标 z。
   */
  Point(decimal_t _x, decimal_t _y, decimal_t _z);

  /**
   * @brief 使用 printf 输出二维坐标，供调试使用。
   */
  void print() const;
};

/// 二维整数像素/栅格坐标，主要用于 OpenCV 地图操作。
struct Point2i {
  int x = 0;
  int y = 0;

  /**
   * @brief 构造位于整数原点的点。
   */
  Point2i();

  /**
   * @brief 使用整数 x、y 构造点。
   *
   * @param _x 横向栅格/像素坐标。
   * @param _y 纵向栅格/像素坐标。
   */
  Point2i(int _x, int _y);

  /**
   * @brief 使用 printf 输出整数坐标，供调试使用。
   */
  void print() const;
};

template <typename T>
/// 将一个几何点与任意数量的同类型属性值绑定。
struct PointWithValue {
  Point pt;
  std::vector<T> values;
};

/**
 * @brief 二维有向包围盒（OBB）。
 *
 * x、y 表示几何中心，angle 表示纵向轴相对世界 x 正方向的航向角，width 和
 * length 分别表示横向宽度与纵向长度。
 */
struct OrientedBoundingBox2D {
  decimal_t x;
  decimal_t y;
  decimal_t angle;
  decimal_t width;
  decimal_t length;

  /**
   * @brief 默认构造；成员值由实现中的默认构造行为决定。
   */
  OrientedBoundingBox2D();

  /**
   * @brief 使用中心、朝向和尺寸构造二维有向包围盒。
   *
   * @param x_ OBB 中心世界坐标 x。
   * @param y_ OBB 中心世界坐标 y。
   * @param angle_ OBB 纵向轴相对世界 x 正方向的角度。
   * @param width_ 横向宽度。
   * @param length_ 纵向长度。
   */
  OrientedBoundingBox2D(const decimal_t x_, const decimal_t y_,
                        const decimal_t angle_, const decimal_t width_,
                        const decimal_t length_);
};

template <int N>
/// 使用中心坐标和各维长度表示的 N 维轴对齐包围盒。
struct AxisAlignedBoundingBoxND {
  std::array<decimal_t, N> coord;
  std::array<decimal_t, N> len;

  /**
   * @brief 默认构造，不主动初始化数组内容。
   */
  AxisAlignedBoundingBoxND() {}

  /**
   * @brief 使用中心坐标和各维总长度构造 AABB。
   *
   * @param coord_ AABB 中心坐标。
   * @param len_ AABB 各维总长度。
   */
  AxisAlignedBoundingBoxND(const std::array<decimal_t, N> coord_,
                           const std::array<decimal_t, N> len_)
      : coord(coord_), len(len_) {}
};

template <typename T, int N_DIM>
/// 使用各维上下界直接表示的 N 维轴对齐超立方体。
struct AxisAlignedCubeNd {
  std::array<T, N_DIM> upper_bound;
  std::array<T, N_DIM> lower_bound;

  /// 默认构造，不主动初始化上下界数组。
  AxisAlignedCubeNd() = default;
  /// 使用上界数组和下界数组构造超立方体。
  AxisAlignedCubeNd(const std::array<T, N_DIM> ub,
                    const std::array<T, N_DIM> lb)
      : upper_bound(ub), lower_bound(lb) {}
};

/// 二维/三维通用圆对象；当前碰撞逻辑主要使用中心的 x、y。
struct Circle {
  Point center;
  decimal_t radius;

  /**
   * @brief 输出圆心和半径，供调试使用。
   */
  void print() const;
};

/// 带方向标记的离散折线，点顺序定义折线拓扑。
struct PolyLine {
  int dir;
  std::vector<Point> points;

  /**
   * @brief 输出方向标记和点数量，供调试使用。
   */
  void print() const;
};

/// 由有序顶点构成的多边形；闭合关系由使用该对象的算法解释。
struct Polygon {
  std::vector<Point> points;

  /**
   * @brief 输出顶点数量，供调试使用。
   */
  void print() const;
};

/**
 * @brief 无状态几何算法集合。
 *
 * OBB 碰撞采用分离轴定理；N 维 AABB 工具以每维上下界为输入。所有输出指针
 * 均由调用方提供有效存储，本类不持有几何对象或缓存。
 */
class ShapeUtils {
 public:
  /**
   * @brief 使用两个 OBB 的四条法向分离轴判断是否相交。
   *
   * @param obb_a 第一个二维有向包围盒。
   * @param obb_b 第二个二维有向包围盒。
   * @return true 所有分离轴上的投影均有正重叠。
   * @return false 至少存在一条无重叠分离轴。
   */
  static bool CheckIfOrientedBoundingBoxIntersect(
      const OrientedBoundingBox2D& obb_a, const OrientedBoundingBox2D& obb_b);

  /**
   * @brief 按固定环绕顺序计算 OBB 的四个顶点。
   *
   * @param obb 输入 OBB。
   * @param vertices 输出顶点容器，函数会先清空并预留四个元素。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetVerticesOfOrientedBoundingBox(
      const OrientedBoundingBox2D& obb, vec_E<Vecf<2>>* vertices);

  /**
   * @brief 从 OBB 顶点的前两条边生成两条单位法向轴。
   *
   * @param vertices 按 OBB 环绕顺序排列的四个顶点。
   * @param axes 输出法向轴；函数在现有内容后追加两个元素。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetPerpendicularAxesOfOrientedBoundingBox(
      const vec_E<Vecf<2>>& vertices, vec_E<Vecf<2>>* axes);

  /**
   * @brief 计算全部顶点在给定轴上的最小/最大标量投影。
   *
   * @param vertices 待投影顶点集合，必须非空。
   * @param axis 投影轴，通常为单位向量。
   * @param proj 输出 `[min, max]` 区间。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetProjectionOnAxis(const vec_E<Vecf<2>>& vertices,
                                       const Vecf<2>& axis, Vecf<2>* proj);

  /**
   * @brief 计算指定顶点到下一顶点边向量的右手单位法向量。
   *
   * @param vertices 至少包含 index+2 个顶点的 OBB 顶点数组。
   * @param index 起始顶点下标；当前断言允许 0..3，但实现会访问 index+1。
   * @param axis 输出二维法向量；退化边输出零向量。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetPerpendicularAxisOfOrientedBoundingBox(
      const vec_E<Vecf<2>>& vertices, const int index, Vecf<2>* axis);

  /**
   * @brief 计算两个一维闭区间的重叠长度。
   *
   * @param a `[min, max]` 形式的第一区间。
   * @param b `[min, max]` 形式的第二区间。
   * @param len 输出重叠长度；分离或仅接触时为 0。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetOverlapLength(const Vecf<2> a, const Vecf<2> b,
                                    decimal_t* len);

  /**
   * @brief 批量把 common::Point2i 转换为 cv::Point2i。
   *
   * @param pts_in 输入整数点数组。
   * @param pts_out 输出 OpenCV 点数组，函数会调整到相同长度。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetCvPoint2iVecUsingCommonPoint2iVec(
      const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>* pts_out);

  /**
   * @brief 把一个 common::Point2i 的坐标复制到 cv::Point2i。
   *
   * @param pt_in 输入公共整数点。
   * @param pt_out 输出 OpenCV 整数点。
   * @return ErrorType 当前实现固定返回 kSuccess。
   */
  static ErrorType GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
                                                  cv::Point2i* pt_out);

  /**
   * @brief 判断超立方体 A 是否在每一维上完全包含超立方体 B。
   *
   * @tparam T 可比较的上下界数值类型。
   * @tparam N_DIM 空间维数。
   * @param cube_a 候选外层立方体。
   * @param cube_b 候选内层立方体。
   * @return true B 的每维上下界均位于 A 内部或边界上。
   * @return false 至少一维越出 A 的范围。
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(
      const common::AxisAlignedCubeNd<T, N_DIM>& cube_a,
      const common::AxisAlignedCubeNd<T, N_DIM>& cube_b) {
    // 包含关系要求所有维度同时满足上下界约束。
    for (int i = 0; i < N_DIM; ++i) {
      if (cube_a.lower_bound[i] > cube_b.lower_bound[i] ||
          cube_a.upper_bound[i] < cube_b.upper_bound[i]) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 判断两个超立方体在全部维度上是否具有正长度重叠。
   *
   * @tparam T 支持加减和数值转换的上下界类型。
   * @tparam N_DIM 空间维数。
   * @param aabb_a 第一个超立方体。
   * @param aabb_b 第二个超立方体。
   * @return true 每一维投影均有正重叠。
   * @return false 至少一维分离或仅边界接触。
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeCollide(
      const common::AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const common::AxisAlignedCubeNd<T, N_DIM>& aabb_b) {
    for (int i = 0; i < N_DIM; ++i) {
      decimal_t half_len_a = fabs(
          static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
          2.0);
      decimal_t half_len_b = fabs(
          static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
          2.0);

      decimal_t center_a =
          static_cast<double>(aabb_a.upper_bound[i] + aabb_a.lower_bound[i]) /
          2.0;
      decimal_t center_b =
          static_cast<double>(aabb_b.upper_bound[i] + aabb_b.lower_bound[i]) /
          2.0;

      decimal_t len_c = fabs(center_a - center_b);

      // 中心距离不小于半长和时，该维分离或仅接触，整体不碰撞。
      if (fabs(half_len_a + half_len_b) <= len_c) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 判断两个非包含 AABB 是否相交，并标记发生穿越的各维表面。
   *
   * @tparam T 上下界数值类型。
   * @tparam N_DIM 空间维数。
   * @param aabb_a 第一个超立方体。
   * @param aabb_b 第二个超立方体。
   * @param inter_dim_a A 的各维上/下表面相交标记，长度为 2*N_DIM。
   * @param inter_dim_b B 的各维上/下表面相交标记，长度为 2*N_DIM。
   * @return true 两者碰撞且不存在完整包含关系。
   * @return false 两者分离、仅接触或一者完整包含另一者。
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdIntersect(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b,
      std::array<bool, N_DIM * 2>* inter_dim_a,
      std::array<bool, N_DIM * 2>* inter_dim_b) {
    inter_dim_a->fill(false);
    inter_dim_b->fill(false);
    // 本接口专门描述表面穿越；完整包含没有穿越表面，因此返回 false。
    if (CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(aabb_a, aabb_b) ||
        CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(aabb_b, aabb_a)) {
      return false;
    }
    // 任一维分离即可提前排除碰撞。
    if (!CheckIfAxisAlignedCubeCollide(aabb_a, aabb_b)) {
      return false;
    }
    // 标记 A 的上界面/下界面是否落入 B 的对应维区间。
    for (int i = 0; i < N_DIM; ++i) {
      if (CheckIfAxisAlignedCubeNdCollideOnOneDim(aabb_a, aabb_b, i)) {
        if (aabb_a.upper_bound[i] < aabb_b.upper_bound[i] &&
            aabb_a.upper_bound[i] > aabb_b.lower_bound[i]) {
          (*inter_dim_a)[2 * i] = true;
        }
        if (aabb_a.lower_bound[i] < aabb_b.upper_bound[i] &&
            aabb_a.lower_bound[i] > aabb_b.lower_bound[i]) {
          (*inter_dim_a)[2 * i + 1] = true;
        }
      }
    }
    // 对称地标记 B 的各维相交表面。
    for (int i = 0; i < N_DIM; ++i) {
      if (CheckIfAxisAlignedCubeNdCollideOnOneDim(aabb_a, aabb_b, i)) {
        if (aabb_b.upper_bound[i] > aabb_a.lower_bound[i] &&
            aabb_b.upper_bound[i] < aabb_a.upper_bound[i]) {
          (*inter_dim_b)[2 * i] = true;
        }
        if (aabb_b.lower_bound[i] > aabb_a.lower_bound[i] &&
            aabb_b.lower_bound[i] < aabb_a.upper_bound[i]) {
          (*inter_dim_b)[2 * i + 1] = true;
        }
      }
    }
    return true;
  }

  /**
   * @brief 判断两个区间在指定维度上是否部分相交。
   * @notice 分离、仅接触或一维区间包含另一维区间时均返回 false。
   * @param aabb_a 第一个超立方体。
   * @param aabb_b 第二个超立方体。
   * @param i 待检查维度下标。
   * @return true 两个一维区间发生非包含的正长度交叠。
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdIntersectionOnOneDim(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b, const int& i) {
    decimal_t half_len_a = fabs(
        static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
        2.0);
    decimal_t half_len_b = fabs(
        static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
        2.0);

    decimal_t center_a = aabb_a.lower_bound[i] + half_len_a;
    decimal_t center_b = aabb_b.lower_bound[i] + half_len_b;
    decimal_t len_c = fabs(center_a - center_b);

    // 半长和用于排除分离，半长差用于排除一方包含另一方。
    if (fabs(half_len_a + half_len_b) <= len_c ||
        fabs(half_len_a - half_len_b) >= len_c) {
      return false;
    }
    return true;
  };

  /**
   * @brief 判断两个区间在指定维度上是否碰撞。
   * @notice 只检查单一维度；包含关系视为碰撞，边界接触视为不碰撞。
   *
   * @tparam T 上下界数值类型。
   * @tparam N_DIM 空间维数。
   * @param aabb_a 第一个超立方体。
   * @param aabb_b 第二个超立方体。
   * @param i 待检查维度下标。
   * @return true 两个一维投影有正重叠。
   * @return false 两个一维投影分离或仅接触。
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdCollideOnOneDim(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b, const int& i) {
    decimal_t half_len_a = fabs(
        static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
        2.0);
    decimal_t half_len_b = fabs(
        static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
        2.0);

    decimal_t center_a = aabb_a.lower_bound[i] + half_len_a;
    decimal_t center_b = aabb_b.lower_bound[i] + half_len_b;
    decimal_t len_c = fabs(center_a - center_b);

    // 这里只排除分离/接触，不排除包含关系。
    if (fabs(half_len_a + half_len_b) <= len_c) {
      return false;
    }
    return true;
  };

};  // class ShapeUtils

}  // namespace common

#endif  //_COMMON_INC_COMMON_BASICS_SHAPES_H__
