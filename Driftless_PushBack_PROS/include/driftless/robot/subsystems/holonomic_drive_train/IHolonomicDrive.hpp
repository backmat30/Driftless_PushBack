#ifndef __I_HOLONOMIC_DRIVE_HPP__
#define __I_HOLONOMIC_DRIVE_HPP__

#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"

namespace driftless {
  namespace robot {
    namespace subsystems {
      namespace holonomic_drive_train {
        class IHolonomicDrive {
          virtual ~IHolonomicDrive() = default;

          virtual void init() = 0;

          virtual void run() = 0;

          virtual HolonomicMotionVector getMotionVector() = 0;

          virtual void setMotionVector(HolonomicMotionVector motionVector) = 0;
        };
      }
    }
  }
}
#endif