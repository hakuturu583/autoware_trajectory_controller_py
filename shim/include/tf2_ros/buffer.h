// Stub tf2_ros::Buffer: there is no TF tree without ROS, every lookup fails.
// The wrapped controllers receive the ego pose through the odometry input instead.
#ifndef SHIM__TF2_ROS__BUFFER_H_
#define SHIM__TF2_ROS__BUFFER_H_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/time.hpp"

#include <memory>
#include <string>

namespace tf2_ros
{
class Buffer
{
public:
  template <class... Args>
  explicit Buffer(Args &&...)
  {
  }
  template <class... Args>
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target, const std::string & source, Args &&...) const
  {
    throw tf2::LookupException(
      "tf is not available in the ROS-free build: " + target + " <- " + source);
  }
  template <class... Args>
  bool canTransform(Args &&...) const
  {
    return false;
  }
  template <class T>
  void setCreateTimerInterface(T &&)
  {
  }
};
}  // namespace tf2_ros

#endif  // SHIM__TF2_ROS__BUFFER_H_
