#ifndef __I_PATH_HPP__
#define __I_PATH_HPP__

#include "driftless/control/Point.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Interface for a generic path
class IPath {
 public:
  /// @brief Destroys the path
  virtual ~IPath() = default;

  /// @brief Gets the point at time t
  /// @param t __double__ The time
  /// @return __Point__ The point at time t
  virtual Point getPoint(double t) const = 0;

  /// @brief Gets the derivative at time t
  /// @param t __double__ The time
  /// @return __Point__ The derivative at time t
  virtual Point getDerivative(double t) const = 0;

  /// @brief Gets the second derivative at time t
  /// @param t __double__ The time
  /// @return __Point__ The second derivative at time t
  virtual Point getSecondDerivative(double t) const = 0;

  /// @brief Gets the maximum time
  /// @return __double__ The maximum time
  virtual double getMaxTime() const = 0;
};
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif