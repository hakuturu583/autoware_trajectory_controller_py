// ROS-free re-implementation of the parts of tf2_geometry_msgs used by Autoware.
#ifndef SHIM__TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_HPP_
#define SHIM__TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_HPP_

// The real header includes tf2_ros (and thereby rclcpp); upstream relies on it.
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/convert.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_ros/time.hpp"  // geometry.cpp uses tf2_ros::fromMsg via this header

namespace tf2
{
// ---- Vector3 / Point ----------------------------------------------------
inline geometry_msgs::msg::Vector3 toMsg(const Vector3 & in)
{
  geometry_msgs::msg::Vector3 out;
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}
inline void fromMsg(const geometry_msgs::msg::Vector3 & in, Vector3 & out)
{
  out = Vector3(in.x, in.y, in.z);
}
inline geometry_msgs::msg::Point & toMsg(const Vector3 & in, geometry_msgs::msg::Point & out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}
inline void fromMsg(const geometry_msgs::msg::Point & in, Vector3 & out)
{
  out = Vector3(in.x, in.y, in.z);
}

// ---- Quaternion ---------------------------------------------------------
inline geometry_msgs::msg::Quaternion toMsg(const Quaternion & in)
{
  geometry_msgs::msg::Quaternion out;
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  out.w = in.w();
  return out;
}
inline void fromMsg(const geometry_msgs::msg::Quaternion & in, Quaternion & out)
{
  out = Quaternion(in.x, in.y, in.z, in.w);
}

// ---- Transform <-> Pose / Transform --------------------------------------
inline geometry_msgs::msg::Transform toMsg(const Transform & in)
{
  geometry_msgs::msg::Transform out;
  out.translation = toMsg(in.getOrigin());
  out.rotation = toMsg(in.getRotation());
  return out;
}
inline void fromMsg(const geometry_msgs::msg::Transform & in, Transform & out)
{
  Vector3 v;
  fromMsg(in.translation, v);
  out.setOrigin(v);
  Quaternion q;
  fromMsg(in.rotation, q);
  out.setRotation(q);
}
inline geometry_msgs::msg::Pose & toMsg(const Transform & in, geometry_msgs::msg::Pose & out)
{
  toMsg(in.getOrigin(), out.position);
  out.orientation = toMsg(in.getRotation());
  return out;
}
inline void fromMsg(const geometry_msgs::msg::Pose & in, Transform & out)
{
  out.setOrigin(Vector3(in.position.x, in.position.y, in.position.z));
  Quaternion q;
  fromMsg(in.orientation, q);
  out.setRotation(q);
}
// Messages are already "converted"; this lets tf2::convert(msg, tf2_type) work.
inline const geometry_msgs::msg::Pose & toMsg(const geometry_msgs::msg::Pose & in) { return in; }
inline const geometry_msgs::msg::Transform & toMsg(const geometry_msgs::msg::Transform & in)
{
  return in;
}
inline const geometry_msgs::msg::Quaternion & toMsg(const geometry_msgs::msg::Quaternion & in)
{
  return in;
}
inline const geometry_msgs::msg::Vector3 & toMsg(const geometry_msgs::msg::Vector3 & in)
{
  return in;
}

template <class A, class B>
void convert(const A & a, B & b)
{
  fromMsg(toMsg(a), b);
}

// ---- doTransform (only the Pose overload is used upstream) ----------------
inline void doTransform(
  const geometry_msgs::msg::Pose & in, geometry_msgs::msg::Pose & out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  Transform t;
  fromMsg(transform.transform, t);
  Transform v;
  fromMsg(in, v);
  toMsg(t * v, out);
}
}  // namespace tf2

#endif  // SHIM__TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_HPP_
