// Minimal ROS-free stand-in for tl::expected (only what the wrapped sources use).
#ifndef SHIM__TL_EXPECTED__EXPECTED_HPP_
#define SHIM__TL_EXPECTED__EXPECTED_HPP_

#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

namespace tl
{
template <class E>
class unexpected
{
public:
  explicit unexpected(E e) : error_(std::move(e)) {}
  const E & value() const & { return error_; }
  E & value() & { return error_; }
  E && value() && { return std::move(error_); }

private:
  E error_;
};

template <class E>
unexpected<std::decay_t<E>> make_unexpected(E && e)
{
  return unexpected<std::decay_t<E>>(std::forward<E>(e));
}

template <class T, class E>
class expected
{
public:
  expected() : storage_(std::in_place_index<0>) {}
  expected(const T & v) : storage_(std::in_place_index<0>, v) {}             // NOLINT
  expected(T && v) : storage_(std::in_place_index<0>, std::move(v)) {}       // NOLINT
  template <
    class U, class = std::enable_if_t<
               std::is_constructible_v<T, U &&> &&
               !std::is_same_v<std::decay_t<U>, expected> &&
               !std::is_same_v<std::decay_t<U>, T>>>
  expected(U && v) : storage_(std::in_place_index<0>, T(std::forward<U>(v)))  // NOLINT
  {
  }
  template <class G>
  expected(const unexpected<G> & u) : storage_(std::in_place_index<1>, u.value())  // NOLINT
  {
  }
  template <class G>
  expected(unexpected<G> && u)  // NOLINT
  : storage_(std::in_place_index<1>, std::move(u).value())
  {
  }

  bool has_value() const { return storage_.index() == 0; }
  explicit operator bool() const { return has_value(); }

  T & value() &
  {
    check();
    return std::get<0>(storage_);
  }
  const T & value() const &
  {
    check();
    return std::get<0>(storage_);
  }
  T && value() &&
  {
    check();
    return std::move(std::get<0>(storage_));
  }
  T & operator*() & { return std::get<0>(storage_); }
  const T & operator*() const & { return std::get<0>(storage_); }
  T * operator->() { return &std::get<0>(storage_); }
  const T * operator->() const { return &std::get<0>(storage_); }
  const E & error() const & { return std::get<1>(storage_); }
  E & error() & { return std::get<1>(storage_); }
  template <class U>
  T value_or(U && u) const &
  {
    return has_value() ? std::get<0>(storage_) : static_cast<T>(std::forward<U>(u));
  }

private:
  void check() const
  {
    if (!has_value()) throw std::runtime_error("bad expected access");
  }
  std::variant<T, E> storage_;
};
}  // namespace tl

#endif  // SHIM__TL_EXPECTED__EXPECTED_HPP_
