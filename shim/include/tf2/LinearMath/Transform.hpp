// ROS-free re-implementation of tf2::Transform (subset).
#ifndef SHIM__TF2__LINEARMATH__TRANSFORM_HPP_
#define SHIM__TF2__LINEARMATH__TRANSFORM_HPP_

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"

namespace tf2
{
class Transform
{
public:
  Transform() = default;
  explicit Transform(const Quaternion & q, const Vector3 & c = Vector3()) : basis_(q), origin_(c) {}
  explicit Transform(const Matrix3x3 & b, const Vector3 & c = Vector3()) : basis_(b), origin_(c) {}

  static const Transform & getIdentity()
  {
    static const Transform t;
    return t;
  }

  void setOrigin(const Vector3 & origin) { origin_ = origin; }
  void setRotation(const Quaternion & q) { basis_.setRotation(q); }
  void setBasis(const Matrix3x3 & b) { basis_ = b; }
  void setIdentity()
  {
    basis_.setIdentity();
    origin_.setZero();
  }
  Vector3 & getOrigin() { return origin_; }
  const Vector3 & getOrigin() const { return origin_; }
  Matrix3x3 & getBasis() { return basis_; }
  const Matrix3x3 & getBasis() const { return basis_; }
  Quaternion getRotation() const
  {
    Quaternion q;
    basis_.getRotation(q);
    return q;
  }

  Vector3 operator()(const Vector3 & x) const { return basis_ * x + origin_; }
  Vector3 operator*(const Vector3 & x) const { return (*this)(x); }
  Quaternion operator*(const Quaternion & q) const { return getRotation() * q; }
  Transform operator*(const Transform & t) const
  {
    return Transform(basis_ * t.basis_, (*this)(t.origin_));
  }
  Transform & operator*=(const Transform & t)
  {
    origin_ = (*this)(t.origin_);
    basis_ = basis_ * t.basis_;
    return *this;
  }
  Transform inverse() const
  {
    const Matrix3x3 inv = basis_.transpose();
    return Transform(inv, inv * -origin_);
  }
  Transform inverseTimes(const Transform & t) const
  {
    const Vector3 v = t.getOrigin() - origin_;
    return Transform(basis_.transpose() * t.basis_, basis_.transpose() * v);
  }

private:
  Matrix3x3 basis_;
  Vector3 origin_;
};
}  // namespace tf2

#endif  // SHIM__TF2__LINEARMATH__TRANSFORM_HPP_
