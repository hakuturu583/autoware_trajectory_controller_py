// Minimal stand-in for fmt::format supporting sequential "{}" placeholders.
#ifndef SHIM__FMT__FORMAT_H_
#define SHIM__FMT__FORMAT_H_

#include <sstream>
#include <string>
#include <utility>

namespace fmt
{
namespace detail
{
inline void format_impl(std::ostringstream & os, const char * f)
{
  for (; *f; ++f) {
    if ((f[0] == '{' && f[1] == '{') || (f[0] == '}' && f[1] == '}')) ++f;
    os << *f;
  }
}
template <class T, class... Rest>
void format_impl(std::ostringstream & os, const char * f, T && v, Rest &&... rest)
{
  for (; *f; ++f) {
    if (f[0] == '{' && f[1] == '{') {
      os << '{';
      ++f;
    } else if (f[0] == '}' && f[1] == '}') {
      os << '}';
      ++f;
    } else if (f[0] == '{') {
      while (*f && *f != '}') ++f;  // ignore format spec
      os << v;
      format_impl(os, *f ? f + 1 : f, std::forward<Rest>(rest)...);
      return;
    } else {
      os << *f;
    }
  }
}
}  // namespace detail

template <class... Args>
std::string format(const std::string & f, Args &&... args)
{
  std::ostringstream os;
  detail::format_impl(os, f.c_str(), std::forward<Args>(args)...);
  return os.str();
}
}  // namespace fmt

#endif  // SHIM__FMT__FORMAT_H_
