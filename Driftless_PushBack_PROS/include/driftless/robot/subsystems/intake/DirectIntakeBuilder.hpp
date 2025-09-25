#ifndef __DIRECT_INTAKE_BUILDER_HPP__
#define __DIRECT_INTAKE_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the intake subsystem
/// @author Matthew Backman
namespace intake {

/// @brief Builder for DirectIntake objects
/// @author Matthew Backman
class DirectIntakeBuilder {
 private:
  /// @brief The front motors used to build the DirectIntake
  hal::MotorGroup m_front_motors{};

  /// @brief The intermediary motors used to build the DirectIntake
  hal::MotorGroup m_intermediary_motors{};

  /// @brief The back motors used to build the DirectIntake
  hal::MotorGroup m_back_motors{};

  /// @brief The pistons used to build the DirectIntake
  hal::PistonGroup m_back_pistons{};

 public:
  /// @brief Add a front motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withFrontMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Add an intermediary motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withIntermediaryMotor(
      std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a back motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withBackMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a back piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>&__ The piston to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withBackPiston(std::unique_ptr<io::IPiston>& piston);

  /// @brief Builds the DirectIntake object
  /// @return __std::unique_ptr<IIntake>__ Pointer to the newly built
  /// DirectIntake
  std::unique_ptr<IIntake> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif