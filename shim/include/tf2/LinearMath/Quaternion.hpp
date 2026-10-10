// ROS-free re-implementation of tf2::Quaternion (subset, same conventions as tf2).
#ifndef SHIM__TF2__LINEARMATH__QUATERNION_HPP_
#define SHIM__TF2__LINEARMATH__QUATERNION_HPP_

#include "tf2/LinearMath/Vector3.hpp"

#include <cmath>

namespace tf2
{
class Quaternion
{
public:
  Quaternion() = default;
  Quaternion(double x, double y, double z, double w) : x_(x), y_(y), z_(z), w_(w) {}
  Quaternion(const Vector3 & axis, double angle) { setRotation(axis, angle); }

  static const Quaternion & getIdentity()
  {
    static const Quaternion q(0.0, 0.0, 0.0, 1.0);
    return q;
  }

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double w() const { return w_; }
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getZ() const { return z_; }
  double getW() const { return w_; }
  void setX(double v) { x_ = v; }
  void setY(double v) { y_ = v; }
  void setZ(double v) { z_ = v; }
  void setW(double v) { w_ = v; }
  void setValue(double x, double y, double z, double w)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
  }

  void setRotation(const Vector3 & axis, double angle)
  {
    const double d = axis.length();
    const double s = std::sin(angle * 0.5) / d;
    setValue(axis.x() * s, axis.y() * s, axis.z() * s, std::cos(angle * 0.5));
  }

  void setRPY(double roll, double pitch, double yaw)
  {
    const double hy = yaw * 0.5, hp = pitch * 0.5, hr = roll * 0.5;
    const double cy = std::cos(hy), sy = std::sin(hy);
    const double cp = std::cos(hp), sp = std::sin(hp);
    const double cr = std::cos(hr), sr = std::sin(hr);
    setValue(
      sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy);
  }
  void setEuler(double yaw, double pitch, double roll)
  {
    const double hy = yaw * 0.5, hp = pitch * 0.5, hr = roll * 0.5;
    const double cy = std::cos(hy), sy = std::sin(hy);
    const double cp = std::cos(hp), sp = std::sin(hp);
    const double cr = std::cos(hr), sr = std::sin(hr);
    setValue(
      cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, sr * cp * cy - cr * sp * sy,
      cr * cp * cy + sr * sp * sy);
  }

  double dot(const Quaternion & q) const
  {
    return x_ * q.x_ + y_ * q.y_ + z_ * q.z_ + w_ * q.w_;
  }
  double length2() const { return dot(*this); }
  double length() const { return std::sqrt(length2()); }
  Quaternion & normalize()
  {
    const double l = length();
    x_ /= l;
    y_ /= l;
    z_ /= l;
    w_ /= l;
    return *this;
  }
  Quaternion normalized() const { return Quaternion(*this).normalize(); }
  Quaternion inverse() const { return Quaternion(-x_, -y_, -z_, w_); }
  double getAngle() const { return 2.0 * std::acos(w_); }
  Vector3 getAxis() const
  {
    const double s2 = 1.0 - w_ * w_;
    if (s2 < 10.0 * 1.19209290e-07) return Vector3(1.0, 0.0, 0.0);
    const double s = 1.0 / std::sqrt(s2);
    return Vector3(x_ * s, y_ * s, z_ * s);
  }
  double angle(const Quaternion & q) const
  {
    const double s = std::sqrt(length2() * q.length2());
    return std::acos(dot(q) / s);
  }
  double angleShortestPath(const Quaternion & q) const
  {
    const double s = std::sqrt(length2() * q.length2());
    if (dot(q) < 0) return std::acos(dot(-q) / s) * 2.0;
    return std::acos(dot(q) / s) * 2.0;
  }

  Quaternion slerp(const Quaternion & q, double t) const
  {
    const double magnitude = std::sqrt(length2() * q.length2());
    const double product = dot(q) / magnitude;
    if (std::fabs(product) < 1.0) {
      const double sign = (product < 0) ? -1.0 : 1.0;
      const double theta = std::acos(sign * product);
      const double s1 = std::sin(sign * t * theta);
      const double d = 1.0 / std::sin(theta);
      const double s0 = std::sin((1.0 - t) * theta);
      return Quaternion(
        (x_ * s0 + q.x() * s1) * d, (y_ * s0 + q.y() * s1) * d, (z_ * s0 + q.z() * s1) * d,
        (w_ * s0 + q.w() * s1) * d);
    }
    return *this;
  }

  Quaternion operator-() const { return Quaternion(-x_, -y_, -z_, -w_); }
  Quaternion operator+(const Quaternion & q) const
  {
    return Quaternion(x_ + q.x_, y_ + q.y_, z_ + q.z_, w_ + q.w_);
  }
  Quaternion operator-(const Quaternion & q) const
  {
    return Quaternion(x_ - q.x_, y_ - q.y_, z_ - q.z_, w_ - q.w_);
  }
  Quaternion operator*(double s) const { return Quaternion(x_ * s, y_ * s, z_ * s, w_ * s); }
  Quaternion & operator*=(const Quaternion & q)
  {
    *this = *this * q;
    return *this;
  }
  Quaternion operator*(const Quaternion & q) const
  {
    return Quaternion(
      w_ * q.x_ + x_ * q.w_ + y_ * q.z_ - z_ * q.y_, w_ * q.y_ + y_ * q.w_ + z_ * q.x_ - x_ * q.z_,
      w_ * q.z_ + z_ * q.w_ + x_ * q.y_ - y_ * q.x_, w_ * q.w_ - x_ * q.x_ - y_ * q.y_ - z_ * q.z_);
  }
  bool operator==(const Quaternion & q) const
  {
    return x_ == q.x_ && y_ == q.y_ && z_ == q.z_ && w_ == q.w_;
  }

private:
  double x_{0.0}, y_{0.0}, z_{0.0}, w_{1.0};
};

inline Quaternion slerp(const Quaternion & q1, const Quaternion & q2, double t)
{
  return q1.slerp(q2, t);
}
inline Quaternion inverse(const Quaternion & q) { return q.inverse(); }
inline double dot(const Quaternion & a, const Quaternion & b) { return a.dot(b); }
inline Vector3 quatRotate(const Quaternion & rotation, const Vector3 & v)
{
  const Quaternion q = rotation * Quaternion(v.x(), v.y(), v.z(), 0.0) * rotation.inverse();
  return Vector3(q.x(), q.y(), q.z());
}
}  // namespace tf2

#endif  // SHIM__TF2__LINEARMATH__QUATERNION_HPP_
