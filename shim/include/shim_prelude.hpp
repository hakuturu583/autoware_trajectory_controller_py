// Force-included into every vendored Autoware translation unit (see CMakeLists.txt).
//
// Upstream sources use these std facilities without including them, relying on what
// rclcpp / tf2 / Eigen pull in transitively in a ROS 2 build. The shim does not
// reproduce those include graphs, so they are provided here once.
#pragma once

#include <cmath>
#include <limits>
#include <math.h>
#include <optional>
