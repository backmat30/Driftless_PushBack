#ifndef __DIRECT_HOOD_BUILDER_HPP__
#define __DIRECT_HOOD_BUILDER_HPP__

#include "driftless/robot/subsystems/hood/DirectHood.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the hood subsystem
/// @author Matthew Backman
namespace hood {

/// @brief Builder for DirectHood objects
/// @author Matthew Backman
class DirectHoodBuilder {
 private:
  /// @brief The motors used to build the DirectHood
  hal::MotorGroup m_motors{};

  /// @brief The height pistons used to build the DirectHood
  hal::PistonGroup m_height_pistons{};

  /// @brief The gate pistons used to build the DirectHood
  hal::PistonGroup m_gate_pistons{};

  /// @brief The descore pistons used to build the DirectHood
  hal::PistonGroup m_descore_pistons{};

  /// @brief The bump pistons used to build the DirectHood
  hal::PistonGroup m_bump_pistons{};

 public:
  /// @brief Adds a motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectHoodBuilder*__ Pointer to this builder
  DirectHoodBuilder* withMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Adds a height piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>&__ The height piston to add
  /// @return __DirectHoodBuilder*__ Pointer to this builder
  DirectHoodBuilder* withHeightPiston(std::unique_ptr<io::IPiston>& piston);

  /// @brief Adds a gate piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>& The gate piston to add
  /// @return __DirectHoodBuilder*__ Pointer to this builder
  DirectHoodBuilder* withGatePiston(std::unique_ptr<io::IPiston>& piston);

  /// @brief Adds a descore piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>& The descore piston to add
  /// @return __DirectHoodBuilder*__ Pointer to this builder
  DirectHoodBuilder* withDescorePiston(std::unique_ptr<io::IPiston>& piston);

  /// @brief Adds a bump piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>& The bump piston to add
  /// @return __DirectHoodBuilder*__ Pointer to this builder
  DirectHoodBuilder* withBumpPiston(std::unique_ptr<io::IPiston>& piston);

  /// @brief Builds the DirectHood object using the supplied values
  /// @return __std::unique_ptr<IHood>__ The DirectHood object that was built
  std::unique_ptr<IHood> build();
};
}  // namespace hood
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif