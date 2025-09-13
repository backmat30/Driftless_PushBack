#ifndef __HOLONOMIC_MOTION_VECTOR_HPP__
#define __HOLONOMIC_MOTION_VECTOR_HPP__

namespace driftless {
  namespace robot {
    namespace subsystems {
      namespace holonomic_drive_train {
        struct HolonomicMotionVector {
          double x_velocity{};
          double y_velocity{};
          double angular_velocity{};
        };
      }
    }
  }
}
#endif