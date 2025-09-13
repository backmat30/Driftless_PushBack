#ifndef __I_HOLONOMIC_DRIVE_HPP__
#define __I_HOLONOMIC_DRIVE_HPP__

#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

  /// @brief The namespace for robot code
  /// @author Matthew Backman
  namespace robot {

    /// @brief The namespace for subsystems code
    /// @author Matthew Backman
    namespace subsystems {

      /// @brief The namespace for the holonomic drive train subsystem code
      /// @author Matthew Backman
      namespace holonomic_drive_train {

        /// @brief Interface for holonomic drive train systems
        /// @author Matthew Backman
        class IHolonomicDrive {
          public:
          /// @brief Destroys the holonomic drive object
          virtual ~IHolonomicDrive() = default;

          /// @brief Initializes the holonomic drive object
          virtual void init() = 0;

          /// @brief Runs the holonomic drive object
          virtual void run() = 0;

          /// @brief Sets the motion vector of the holonomic drive train
          /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
          virtual void setMotionVector(HolonomicMotionVector motion_ector) = 0;

          /// @brief Sets the motion vector of the holonomic drive train with normalized vector
          /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
          /// @details Magnitude must be within [0, 1]
          /// @details Angular velocity must be within [-1, 1]
          virtual void setNormalizedMotionVector(HolonomicMotionVector motion_vector) = 0;
        };
      }
    }
  }
}
#endif