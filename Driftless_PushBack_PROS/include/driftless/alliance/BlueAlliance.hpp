#ifndef __BLUE_ALLIANCE_HPP__
#define __BLUE_ALLIANCE_HPP__

#include "driftless/alliance/IAlliance.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for alliances
/// @author Matthew Backman
namespace alliance {

/// @brief Class representing the blue alliance
/// @author Matthew Backman
class BlueAlliance : public IAlliance {
 public:
  /// @brief Gets the enumerated value of the alliance
  /// @return __EAlliance__ The alliance
  EAlliance getAlliance() override;

  /// @brief Gets the name of the alliance
  /// @return __std::string__ The name of the alliance
  std::string getName() override;
};
}  // namespace alliance
}  // namespace driftless
#endif