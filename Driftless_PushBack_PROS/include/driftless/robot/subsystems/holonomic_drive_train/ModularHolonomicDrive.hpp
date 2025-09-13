#ifndef __MODULAR_HOLONOMIC_DRIVE_HPP__
#define __MODULAR_HOLONOMIC_DRIVE_HPP__

#include "driftless/robot/subsystems/holonomic_drive_train/IHolonomicDrive.hpp"

namespace driftless {
  namespace robot {
    namespace subsystems {
      namespace holonomic_drive_train {
        class ModularHolonomicDrive : public IHolonomicDrive {
          private:
          public:
            void init() override;

            void run() override;

            HolonomicMotionVector getMotionVector() override;

            void setMotionVector(HolonomicMotionVector motionVector) override;
        };
      }
    }
  }
}
#endif