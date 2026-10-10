#ifndef SHIM__TF2__TIME_HPP_
#define SHIM__TF2__TIME_HPP_

#include <chrono>

namespace tf2
{
using Duration = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<std::chrono::system_clock, Duration>;
static const TimePoint TimePointZero = TimePoint(Duration::zero());

inline double timeToSec(const TimePoint & t)
{
  return std::chrono::duration<double>(t.time_since_epoch()).count();
}
inline double durationToSec(const Duration & d) { return std::chrono::duration<double>(d).count(); }
inline TimePoint get_now()
{
  return std::chrono::time_point_cast<Duration>(std::chrono::system_clock::now());
}
}  // namespace tf2

#endif  // SHIM__TF2__TIME_HPP_
