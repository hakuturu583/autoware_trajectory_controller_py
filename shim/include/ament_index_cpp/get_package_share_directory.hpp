// Package share lookup without ament: searches the directories listed in the
// AUTOWARE_TC_SHARE_DIRS environment variable (":"-separated) for `<root>/<package>`.
#ifndef SHIM__AMENT_INDEX_CPP__GET_PACKAGE_SHARE_DIRECTORY_HPP_
#define SHIM__AMENT_INDEX_CPP__GET_PACKAGE_SHARE_DIRECTORY_HPP_

#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

namespace ament_index_cpp
{
class PackageNotFoundError : public std::out_of_range
{
public:
  explicit PackageNotFoundError(const std::string & name)
  : std::out_of_range("package '" + name + "' not found (set AUTOWARE_TC_SHARE_DIRS)")
  {
  }
};

inline std::string get_package_share_directory(const std::string & package_name)
{
  const char * env = std::getenv("AUTOWARE_TC_SHARE_DIRS");
  std::stringstream ss(env ? env : "");
  std::string root;
  while (std::getline(ss, root, ':')) {
    const auto dir = std::filesystem::path(root) / package_name;
    if (!root.empty() && std::filesystem::is_directory(dir)) return dir.string();
  }
  throw PackageNotFoundError(package_name);
}
}  // namespace ament_index_cpp

#endif  // SHIM__AMENT_INDEX_CPP__GET_PACKAGE_SHARE_DIRECTORY_HPP_
