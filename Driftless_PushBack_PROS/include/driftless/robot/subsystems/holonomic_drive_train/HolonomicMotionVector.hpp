#ifndef __HOLONOMIC_MOTION_VECTOR_HPP__
#define __HOLONOMIC_MOTION_VECTOR_HPP__

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

        /// @brief Represents a motion vector for a holonomic drive train
        /// @author Matthew Backman
        struct HolonomicMotionVector {
          /// @brief The x-component of the motion vector (strafe)
          double x{};

          /// @brief The y-component of the motion vector (forward)
          double y{};

          /// @brief The angular component of the motion vector (rotation)
          double angular_velocity{};
        };
      }
    }
  }
}
#endif