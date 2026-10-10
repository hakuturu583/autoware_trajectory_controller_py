#ifndef SHIM__TF2__CONVERT_HPP_
#define SHIM__TF2__CONVERT_HPP_

#include "tf2/transform_datatypes.hpp"

namespace tf2
{
/// Generic conversion: `a` is converted to a message with toMsg() (unless it already
/// is one) and then read into `b` with fromMsg(). Overloads live in tf2_geometry_msgs.
template <class A, class B>
void convert(const A & a, B & b);
}  // namespace tf2

#endif  // SHIM__TF2__CONVERT_HPP_
