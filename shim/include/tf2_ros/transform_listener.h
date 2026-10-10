#ifndef SHIM__TF2_ROS__TRANSFORM_LISTENER_H_
#define SHIM__TF2_ROS__TRANSFORM_LISTENER_H_

#include "tf2_ros/buffer.h"

namespace tf2_ros
{
class TransformListener
{
public:
  template <class... Args>
  explicit TransformListener(Buffer &, Args &&...)
  {
  }
};
}  // namespace tf2_ros

#endif  // SHIM__TF2_ROS__TRANSFORM_LISTENER_H_
