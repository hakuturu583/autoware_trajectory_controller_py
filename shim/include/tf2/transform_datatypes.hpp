#ifndef SHIM__TF2__TRANSFORM_DATATYPES_HPP_
#define SHIM__TF2__TRANSFORM_DATATYPES_HPP_

#include "tf2/time.hpp"

#include <string>

namespace tf2
{
template <typename T>
class Stamped : public T
{
public:
  TimePoint stamp_{};
  std::string frame_id_{};

  Stamped() = default;
  Stamped(const T & input, const TimePoint & timestamp, const std::string & frame_id)
  : T(input), stamp_(timestamp), frame_id_(frame_id)
  {
  }
  void setData(const T & input) { *static_cast<T *>(this) = input; }
};
}  // namespace tf2

#endif  // SHIM__TF2__TRANSFORM_DATATYPES_HPP_
