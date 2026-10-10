// ROS-free re-implementation of tf2::Matrix3x3 (subset, same conventions as tf2).
#ifndef SHIM__TF2__LINEARMATH__MATRIX3X3_HPP_
#define SHIM__TF2__LINEARMATH__MATRIX3X3_HPP_

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include <cmath>

namespace tf2
{
class Matrix3x3
{
public:
  Matrix3x3() = default;
  explicit Matrix3x3(const Quaternion & q) { setRotation(q); }
  Matrix3x3(
    double xx, double xy, double xz, double yx, double yy, double yz, double zx, double zy,
    double zz)
  {
    setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
  }

  void setValue(
    double xx, double xy, double xz, double yx, double yy, double yz, double zx, double zy,
    double zz)
  {
    el_[0].setValue(xx, xy, xz);
    el_[1].setValue(yx, yy, yz);
    el_[2].setValue(zx, zy, zz);
  }

  void setRotation(const Quaternion & q)
  {
    const double d = q.length2();
    const double s = 2.0 / d;
    const double xs = q.x() * s, ys = q.y() * s, zs = q.z() * s;
    const double wx = q.w() * xs, wy = q.w() * ys, wz = q.w() * zs;
    const double xx = q.x() * xs, xy = q.x() * ys, xz = q.x() * zs;
    const double yy = q.y() * ys, yz = q.y() * zs, zz = q.z() * zs;
    setValue(
      1.0 - (yy + zz), xy - wz, xz + wy, xy + wz, 1.0 - (xx + zz), yz - wx, xz - wy, yz + wx,
      1.0 - (xx + yy));
  }

  void setRPY(double roll, double pitch, double yaw) { setEulerYPR(yaw, pitch, roll); }
  void setEulerYPR(double yaw, double pitch, double roll)
  {
    const double ci = std::cos(roll), cj = std::cos(pitch), ch = std::cos(yaw);
    const double si = std::sin(roll), sj = std::sin(pitch), sh = std::sin(yaw);
    const double cc = ci * ch, cs = ci * sh, sc = si * ch, ss = si * sh;
    setValue(
      cj * ch, sj * sc - cs, sj * cc + ss, cj * sh, sj * ss + cc, sj * cs - sc, -sj, cj * si,
      cj * ci);
  }
  void setIdentity() { setValue(1, 0, 0, 0, 1, 0, 0, 0, 1); }

  void getRotation(Quaternion & q) const
  {
    const double trace = el_[0].x() + el_[1].y() + el_[2].z();
    double temp[4];
    if (trace > 0.0) {
      double s = std::sqrt(trace + 1.0);
      temp[3] = s * 0.5;
      s = 0.5 / s;
      temp[0] = (el_[2].y() - el_[1].z()) * s;
      temp[1] = (el_[0].z() - el_[2].x()) * s;
      temp[2] = (el_[1].x() - el_[0].y()) * s;
    } else {
      const int i = el_[0].x() < el_[1].y() ? (el_[1].y() < el_[2].z() ? 2 : 1)
                                            : (el_[0].x() < el_[2].z() ? 2 : 0);
      const int j = (i + 1) % 3;
      const int k = (i + 2) % 3;
      double s = std::sqrt(el_[i][i] - el_[j][j] - el_[k][k] + 1.0);
      temp[i] = s * 0.5;
      s = 0.5 / s;
      temp[3] = (el_[k][j] - el_[j][k]) * s;
      temp[j] = (el_[j][i] + el_[i][j]) * s;
      temp[k] = (el_[k][i] + el_[i][k]) * s;
    }
    q.setValue(temp[0], temp[1], temp[2], temp[3]);
  }

  void getEulerYPR(double & yaw, double & pitch, double & roll, unsigned int solution_number = 1) const
  {
    struct Euler
    {
      double yaw, pitch, roll;
    } e1{}, e2{};
    if (std::fabs(el_[2].x()) >= 1) {
      e1.yaw = 0;
      e2.yaw = 0;
      const double delta = std::atan2(el_[2].y(), el_[2].z());
      if (el_[2].x() < 0) {
        e1.pitch = M_PI / 2.0;
        e2.pitch = M_PI / 2.0;
        e1.roll = delta;
        e2.roll = delta;
      } else {
        e1.pitch = -M_PI / 2.0;
        e2.pitch = -M_PI / 2.0;
        e1.roll = delta;
        e2.roll = delta;
      }
    } else {
      e1.pitch = -std::asin(el_[2].x());
      e2.pitch = M_PI - e1.pitch;
      e1.roll = std::atan2(el_[2].y() / std::cos(e1.pitch), el_[2].z() / std::cos(e1.pitch));
      e2.roll = std::atan2(el_[2].y() / std::cos(e2.pitch), el_[2].z() / std::cos(e2.pitch));
      e1.yaw = std::atan2(el_[1].x() / std::cos(e1.pitch), el_[0].x() / std::cos(e1.pitch));
      e2.yaw = std::atan2(el_[1].x() / std::cos(e2.pitch), el_[0].x() / std::cos(e2.pitch));
    }
    const Euler & e = (solution_number == 1) ? e1 : e2;
    yaw = e.yaw;
    pitch = e.pitch;
    roll = e.roll;
  }
  void getRPY(double & roll, double & pitch, double & yaw, unsigned int solution_number = 1) const
  {
    getEulerYPR(yaw, pitch, roll, solution_number);
  }

  const Vector3 & getRow(int i) const { return el_[i]; }
  Vector3 getColumn(int i) const { return Vector3(el_[0][i], el_[1][i], el_[2][i]); }
  Vector3 & operator[](int i) { return el_[i]; }
  const Vector3 & operator[](int i) const { return el_[i]; }

  Matrix3x3 transpose() const
  {
    return Matrix3x3(
      el_[0].x(), el_[1].x(), el_[2].x(), el_[0].y(), el_[1].y(), el_[2].y(), el_[0].z(),
      el_[1].z(), el_[2].z());
  }
  Vector3 operator*(const Vector3 & v) const
  {
    return Vector3(el_[0].dot(v), el_[1].dot(v), el_[2].dot(v));
  }
  Matrix3x3 operator*(const Matrix3x3 & m) const
  {
    const Matrix3x3 t = m.transpose();
    return Matrix3x3(
      el_[0].dot(t[0]), el_[0].dot(t[1]), el_[0].dot(t[2]), el_[1].dot(t[0]), el_[1].dot(t[1]),
      el_[1].dot(t[2]), el_[2].dot(t[0]), el_[2].dot(t[1]), el_[2].dot(t[2]));
  }

private:
  Vector3 el_[3]{Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)};
};
}  // namespace tf2

#endif  // SHIM__TF2__LINEARMATH__MATRIX3X3_HPP_
