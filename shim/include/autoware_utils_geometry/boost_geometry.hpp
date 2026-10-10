// Boost-free replacement of autoware_utils_geometry/boost_geometry.hpp.
//
// The original header registers Point2d/Point3d with boost::geometry and defines
// polygon/linestring aliases on top of boost::geometry models. The wrapped controllers
// only need the point types, so this replacement keeps Point2d/Point3d identical and
// provides plain std::vector based containers for the other aliases.
#ifndef AUTOWARE_UTILS_GEOMETRY__BOOST_GEOMETRY_HPP_
#define AUTOWARE_UTILS_GEOMETRY__BOOST_GEOMETRY_HPP_

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include <geometry_msgs/msg/point.hpp>

#include <vector>

namespace autoware_utils_geometry
{
struct Point2d;
struct Point3d;

struct Point2d : public Eigen::Vector2d
{
  Point2d() : Eigen::Vector2d(0.0, 0.0) {}
  Point2d(const double x, const double y) : Eigen::Vector2d(x, y) {}

  [[nodiscard]] Point3d to_3d(const double z = 0.0) const;
};

struct Point3d : public Eigen::Vector3d
{
  Point3d() : Eigen::Vector3d(0.0, 0.0, 0.0) {}
  Point3d(const double x, const double y, const double z) : Eigen::Vector3d(x, y, z) {}

  [[nodiscard]] Point2d to_2d() const;
};

inline Point3d Point2d::to_3d(const double z) const { return Point3d{x(), y(), z}; }
inline Point2d Point3d::to_2d() const { return Point2d{x(), y()}; }

template <class PointT>
using LineString = std::vector<PointT>;
template <class PointT>
struct Polygon
{
  std::vector<PointT> outer_;
  std::vector<std::vector<PointT>> inners_;
  std::vector<PointT> & outer() { return outer_; }
  const std::vector<PointT> & outer() const { return outer_; }
  std::vector<std::vector<PointT>> & inners() { return inners_; }
  const std::vector<std::vector<PointT>> & inners() const { return inners_; }
};

using LineString2d = LineString<Point2d>;
using LinearRing2d = std::vector<Point2d>;
using Line2d = LineString<Point2d>;
using Polygon2d = Polygon<Point2d>;
using MultiPoint2d = std::vector<Point2d>;
using MultiLineString2d = std::vector<LineString2d>;
using MultiPolygon2d = std::vector<Polygon2d>;

using LineString3d = LineString<Point3d>;
using LinearRing3d = std::vector<Point3d>;
using Polygon3d = Polygon<Point3d>;
using MultiPoint3d = std::vector<Point3d>;
using MultiLineString3d = std::vector<LineString3d>;
using MultiPolygon3d = std::vector<Polygon3d>;

inline geometry_msgs::msg::Point to_msg(const Point3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

inline Point3d from_msg(const geometry_msgs::msg::Point & msg)
{
  Point3d point;
  point.x() = msg.x;
  point.y() = msg.y;
  point.z() = msg.z;
  return point;
}
}  // namespace autoware_utils_geometry

#endif  // AUTOWARE_UTILS_GEOMETRY__BOOST_GEOMETRY_HPP_
