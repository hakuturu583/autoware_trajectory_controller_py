// Pieces of autoware_utils that depend on boost::geometry, replaced for the ROS-free build.
#include "autoware_utils_geometry/gjk_2d.hpp"

#include <stdexcept>

namespace autoware_utils_geometry::gjk
{
bool intersects(const Polygon2d &, const Polygon2d &)
{
  throw std::logic_error("gjk::intersects is not available in the ROS-free build");
}
}  // namespace autoware_utils_geometry::gjk
