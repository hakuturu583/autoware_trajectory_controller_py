// Minimal stand-in for boost::optional implemented with std::optional.
#ifndef SHIM__BOOST__OPTIONAL_HPP_
#define SHIM__BOOST__OPTIONAL_HPP_

#include <optional>
#include <utility>

namespace boost
{
struct none_t
{
  constexpr operator std::nullopt_t() const { return std::nullopt; }  // NOLINT
};
inline constexpr none_t none{};

template <class T>
class optional : public std::optional<T>
{
public:
  using std::optional<T>::optional;
  optional() = default;
  optional(none_t) : std::optional<T>() {}  // NOLINT
  optional(const std::optional<T> & o) : std::optional<T>(o) {}  // NOLINT
  T & get() { return this->value(); }
  const T & get() const { return this->value(); }
  bool is_initialized() const { return this->has_value(); }
  T * get_ptr() { return this->has_value() ? &**this : nullptr; }
  const T * get_ptr() const { return this->has_value() ? &**this : nullptr; }
  bool operator!() const { return !this->has_value(); }
};

template <class T>
optional<std::decay_t<T>> make_optional(T && v)
{
  return optional<std::decay_t<T>>(std::forward<T>(v));
}
}  // namespace boost

#endif  // SHIM__BOOST__OPTIONAL_HPP_
