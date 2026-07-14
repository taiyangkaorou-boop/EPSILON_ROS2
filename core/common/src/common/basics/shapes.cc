/**
 * @file shapes.cc
 * @author HKUST Aerial Robotics Group
 * @brief 实现基础几何对象构造、调试输出、分离轴碰撞检测和 OpenCV 点转换。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */

#include "common/basics/shapes.h"

namespace common {

// Point 构造函数依赖头文件中的成员默认值初始化未提供的坐标分量。
Point::Point() {}

Point::Point(decimal_t _x, decimal_t _y) : x(_x), y(_y) {}

Point::Point(decimal_t _x, decimal_t _y, decimal_t _z) : x(_x), y(_y), z(_z) {}

// 调试输出刻意只显示 x、y，与当前二维规划/碰撞场景保持一致。
void Point::print() const { printf("(%f, %f)", x, y); }

Point2i::Point2i() {}

Point2i::Point2i(int _x, int _y) : x(_x), y(_y) {}

// 输出整数二维点坐标。
void Point2i::print() const { printf("(%d, %d)", x, y); }

OrientedBoundingBox2D::OrientedBoundingBox2D() {}

OrientedBoundingBox2D::OrientedBoundingBox2D(const decimal_t x_,
                                             const decimal_t y_,
                                             const decimal_t angle_,
                                             const decimal_t width_,
                                             const decimal_t length_)
    : x(x_), y(y_), angle(angle_), width(width_), length(length_) {}

// 输出圆心和半径调试信息。
void Circle::print() const {
  printf("Circle:\n");
  printf(" -- center:");
  center.print();
  printf("\n -- radius: %lf\n", radius);
}

// 输出折线方向和点数量调试信息。
void PolyLine::print() const {
  printf("PolyLine:\n");
  printf(" -- dir: %d", dir);
  printf(" -- num of pts: %d", (int)points.size());
}

// 输出多边形点数量调试信息。
void Polygon::print() const {
  printf("Polygon:\n");
  printf(" -- num of pts: %d", (int)points.size());
}

bool ShapeUtils::CheckIfOrientedBoundingBoxIntersect(
    const OrientedBoundingBox2D& obb_a, const OrientedBoundingBox2D& obb_b) {
  // 分离轴定理：生成两个 OBB 的顶点，并收集两组边法向作为候选分离轴。
  vec_E<Vecf<2>> vertices_a, vertices_b;
  GetVerticesOfOrientedBoundingBox(obb_a, &vertices_a);
  GetVerticesOfOrientedBoundingBox(obb_b, &vertices_b);
  vec_E<Vecf<2>> axes;
  GetPerpendicularAxesOfOrientedBoundingBox(vertices_a, &axes);
  GetPerpendicularAxesOfOrientedBoundingBox(vertices_b, &axes);
  Vecf<2> proj_a, proj_b;
  decimal_t overlap_len;
  // decimal_t minoverlap = std::numeric_limits<decimal_t>::infinity();
  // 任一轴投影无正重叠即可判定分离；全部轴重叠才判定碰撞。
  for (auto& axis : axes) {
    GetProjectionOnAxis(vertices_a, axis, &proj_a);
    GetProjectionOnAxis(vertices_b, axis, &proj_b);
    GetOverlapLength(proj_a, proj_b, &overlap_len);
    if (fabs(overlap_len) < kEPS) {  // shapes are not overlapping
      return false;
    }
  }
  return true;
}

ErrorType ShapeUtils::GetVerticesOfOrientedBoundingBox(
    const OrientedBoundingBox2D& obb, vec_E<Vecf<2>>* vertices) {
  // 顶点按 corner1→corner2→corner3→corner4 环绕排列，供边向量计算复用。
  //    corner2   corner1
  // y<--   ________   x
  //        |      |   ^
  //        |      |   |
  //        |      |
  //        |______|
  //    corner3   corner4
  vertices->clear();
  vertices->reserve(4);
  // 将车体局部纵/横半轴旋转到世界坐标后叠加到几何中心。
  decimal_t cos_theta = cos(obb.angle);
  decimal_t sin_theta = sin(obb.angle);
  Vecf<2> corner1(
      obb.x + 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta,
      obb.y + 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
  Vecf<2> corner2(
      obb.x + 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta,
      obb.y + 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
  Vecf<2> corner3(
      obb.x - 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta,
      obb.y - 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
  Vecf<2> corner4(
      obb.x - 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta,
      obb.y - 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
  vertices->push_back(corner1);
  vertices->push_back(corner2);
  vertices->push_back(corner3);
  vertices->push_back(corner4);
  return kSuccess;
}

ErrorType ShapeUtils::GetPerpendicularAxesOfOrientedBoundingBox(
    const vec_E<Vecf<2>>& vertices, vec_E<Vecf<2>>* axes) {
  // 矩形只有两组互相平行的边，因此每个 OBB 只需两条独立法向轴。
  Vecf<2> axis0, axis1;
  GetPerpendicularAxisOfOrientedBoundingBox(vertices, 0, &axis0);
  GetPerpendicularAxisOfOrientedBoundingBox(vertices, 1, &axis1);
  axes->push_back(axis0);
  axes->push_back(axis1);
  return kSuccess;
}

ErrorType ShapeUtils::GetProjectionOnAxis(const vec_E<Vecf<2>>& vertices,
                                          const Vecf<2>& axis, Vecf<2>* proj) {
  // 扫描全部顶点的点积，形成该几何体在轴上的一维包围区间。
  decimal_t min = std::numeric_limits<decimal_t>::infinity();
  decimal_t max = -std::numeric_limits<decimal_t>::infinity();
  decimal_t projection;
  for (auto& vertex : vertices) {
    projection = vertex.dot(axis);
    if (projection < min) {
      min = projection;
    }
    if (projection > max) {
      max = projection;
    }
  }
  *proj = Vecf<2>(min, max);
  return kSuccess;
}

ErrorType ShapeUtils::GetPerpendicularAxisOfOrientedBoundingBox(
    const vec_E<Vecf<2>>& vertices, const int index, Vecf<2>* axis) {
  // 使用相邻顶点形成边向量；调用者必须保证 index+1 不越界。
  assert(index >= 0 && index < 4);
  Vecf<2> vec = vertices[index + 1] - vertices[index];
  decimal_t length = vec.norm();
  Vecf<2> normalized_vec = Vecf<2>::Zero();
  if (length > kEPS) normalized_vec = vec / length;
  // 将单位边向量逆时针旋转 90 度，得到右手二维法向表示。
  (*axis)[0] = -normalized_vec[1];
  (*axis)[1] = normalized_vec[0];
  return kSuccess;
}

ErrorType ShapeUtils::GetOverlapLength(const Vecf<2> a, const Vecf<2> b,
                                       decimal_t* len) {
  // 两区间分离时直接返回零；端点相接也会在后续计算中得到零长度。
  if (a.x() > b.y() || a.y() < b.x()) {
    *len = 0.0;
    return kSuccess;
  }

  // 交集上界减交集下界即为重叠长度。
  *len = std::min(a.y(), b.y()) - std::max(a.x(), b.x());
  return kSuccess;
}

ErrorType ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(
    const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>* pts_out) {
  // 预先调整输出长度，然后逐点复制，保持输入顺序不变。
  int num = pts_in.size();
  pts_out->resize(num);
  for (int i = 0; i < num; ++i) {
    GetCvPoint2iUsingCommonPoint2i(pts_in[i], pts_out->data() + i);
  }
  return kSuccess;
}

ErrorType ShapeUtils::GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
                                                     cv::Point2i* pt_out) {
  // 两种点类型坐标语义一致，无需缩放或坐标轴变换。
  pt_out->x = pt_in.x;
  pt_out->y = pt_in.y;
  return kSuccess;
}

}  // namespace common
