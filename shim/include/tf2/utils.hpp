// ROS-free re-implementation of tf2/utils.hpp (getYaw / getEulerYPR / ...).
#ifndef SHIM__TF2__UTILS_HPP_
#define SHIM__TF2__UTILS_HPP_

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/convert.hpp"

#include <cmath>

namespace tf2
{
namespace impl
{
inline Quaternion toQuaternion(const Quaternion & q) { return q; }
inline Quaternion toQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return Quaternion(q.x, q.y, q.z, q.w);
}
template <typename T>
Quaternion toQuaternion(const T & t)  // e.g. geometry_msgs::msg::Pose
{
  return toQuaternion(t.orientation);
}

inline double getYaw(const Quaternion & q)
{
  double yaw;
  const double sqw = q.w() * q.w();
  const double sqx = q.x() * q.x();
  const double sqy = q.y() * q.y();
  const double sqz = q.z() * q.z();
  const double sarg = -2 * (q.x() * q.z() - q.w() * q.y()) / (sqx + sqy + sqz + sqw);
  if (sarg <= -0.99999) {
    yaw = -2 * std::atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    yaw = 2 * std::atan2(q.y(), q.x());
  } else {
    yaw = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
  }
  return yaw;
}

inline void getEulerYPR(const Quaternion & q, double & yaw, double & pitch, double & roll)
{
  const double sqw = q.w() * q.w();
  const double sqx = q.x() * q.x();
  const double sqy = q.y() * q.y();
  const double sqz = q.z() * q.z();
  const double sarg = -2 * (q.x() * q.z() - q.w() * q.y()) / (sqx + sqy + sqz + sqw);
  if (sarg <= -0.99999) {
    pitch = -0.5 * M_PI;
    roll = 0;
    yaw = -2 * std::atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    pitch = 0.5 * M_PI;
    roll = 0;
    yaw = 2 * std::atan2(q.y(), q.x());
  } else {
    pitch = std::asin(sarg);
    roll = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), sqw - sqx - sqy + sqz);
    yaw = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
  }
}
}  // namespace impl

template <class A>
double getYaw(const A & a)
{
  return impl::getYaw(impl::toQuaternion(a));
}

template <class A>
void getEulerYPR(const A & a, double & yaw, double & pitch, double & roll)
{
  impl::getEulerYPR(impl::toQuaternion(a), yaw, pitch, roll);
}

template <class A>
A getTransformIdentity()
{
  return A();
}
}  // namespace tf2

#endif  // SHIM__TF2__UTILS_HPP_
