// ROS-free re-implementation of tf2::Vector3 (subset).
#ifndef SHIM__TF2__LINEARMATH__VECTOR3_HPP_
#define SHIM__TF2__LINEARMATH__VECTOR3_HPP_

#include <cmath>

namespace tf2
{
using tf2Scalar = double;

class Vector3
{
public:
  Vector3() = default;
  Vector3(double x, double y, double z) : v_{x, y, z} {}

  double x() const { return v_[0]; }
  double y() const { return v_[1]; }
  double z() const { return v_[2]; }
  double getX() const { return v_[0]; }
  double getY() const { return v_[1]; }
  double getZ() const { return v_[2]; }
  void setX(double x) { v_[0] = x; }
  void setY(double y) { v_[1] = y; }
  void setZ(double z) { v_[2] = z; }
  void setValue(double x, double y, double z)
  {
    v_[0] = x;
    v_[1] = y;
    v_[2] = z;
  }
  void setZero() { setValue(0.0, 0.0, 0.0); }
  double & operator[](int i) { return v_[i]; }
  const double & operator[](int i) const { return v_[i]; }

  double dot(const Vector3 & o) const { return x() * o.x() + y() * o.y() + z() * o.z(); }
  Vector3 cross(const Vector3 & o) const
  {
    return Vector3(
      y() * o.z() - z() * o.y(), z() * o.x() - x() * o.z(), x() * o.y() - y() * o.x());
  }
  double length2() const { return dot(*this); }
  double length() const { return std::sqrt(length2()); }
  double distance2(const Vector3 & o) const { return (o - *this).length2(); }
  double distance(const Vector3 & o) const { return (o - *this).length(); }
  Vector3 & normalize() { return *this /= length(); }
  Vector3 normalized() const { return *this / length(); }
  Vector3 absolute() const { return Vector3(std::fabs(x()), std::fabs(y()), std::fabs(z())); }
  double angle(const Vector3 & o) const
  {
    const double s = std::sqrt(length2() * o.length2());
    return std::acos(dot(o) / s);
  }
  Vector3 lerp(const Vector3 & o, double t) const
  {
    return Vector3(
      x() + (o.x() - x()) * t, y() + (o.y() - y()) * t, z() + (o.z() - z()) * t);
  }

  Vector3 & operator+=(const Vector3 & o)
  {
    v_[0] += o.v_[0];
    v_[1] += o.v_[1];
    v_[2] += o.v_[2];
    return *this;
  }
  Vector3 & operator-=(const Vector3 & o)
  {
    v_[0] -= o.v_[0];
    v_[1] -= o.v_[1];
    v_[2] -= o.v_[2];
    return *this;
  }
  Vector3 & operator*=(double s)
  {
    v_[0] *= s;
    v_[1] *= s;
    v_[2] *= s;
    return *this;
  }
  Vector3 & operator/=(double s) { return *this *= (1.0 / s); }
  Vector3 operator+(const Vector3 & o) const { return Vector3(*this) += o; }
  Vector3 operator-(const Vector3 & o) const { return Vector3(*this) -= o; }
  Vector3 operator-() const { return Vector3(-x(), -y(), -z()); }
  Vector3 operator*(double s) const { return Vector3(*this) *= s; }
  Vector3 operator/(double s) const { return Vector3(*this) /= s; }
  bool operator==(const Vector3 & o) const { return x() == o.x() && y() == o.y() && z() == o.z(); }
  bool operator!=(const Vector3 & o) const { return !(*this == o); }

private:
  double v_[3]{0.0, 0.0, 0.0};
};

inline Vector3 operator*(double s, const Vector3 & v) { return v * s; }
inline double tf2Dot(const Vector3 & a, const Vector3 & b) { return a.dot(b); }
inline Vector3 tf2Cross(const Vector3 & a, const Vector3 & b) { return a.cross(b); }
inline Vector3 lerp(const Vector3 & a, const Vector3 & b, double t) { return a.lerp(b, t); }
}  // namespace tf2

#endif  // SHIM__TF2__LINEARMATH__VECTOR3_HPP_
