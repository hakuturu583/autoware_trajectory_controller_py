#ifndef SHIM__TF2__EXCEPTIONS_HPP_
#define SHIM__TF2__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace tf2
{
class TransformException : public std::runtime_error
{
public:
  explicit TransformException(const std::string & what) : std::runtime_error(what) {}
};
class LookupException : public TransformException
{
public:
  using TransformException::TransformException;
};
class ConnectivityException : public TransformException
{
public:
  using TransformException::TransformException;
};
class ExtrapolationException : public TransformException
{
public:
  using TransformException::TransformException;
};
}  // namespace tf2

#endif  // SHIM__TF2__EXCEPTIONS_HPP_
