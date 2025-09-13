#ifndef __MODULAR_HOLONOMIC_DRIVE_HPP__
#define __MODULAR_HOLONOMIC_DRIVE_HPP__

#include <memory>
#include <vector>

#include "driftless/robot/subsystems/holonomic_drive_train/IHolonomicDrive.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/IHolonomicDriveModule.hpp"

namespace driftless {
  namespace robot {
    namespace subsystems {
      namespace holonomic_drive_train {
        class ModularHolonomicDrive : public IHolonomicDrive {
          private:
          std::vector<std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>> m_modules;
          public:
            void init() override;

            void run() override;

            HolonomicMotionVector getMotionVector() override;

            void setMotionVector(HolonomicMotionVector motionVector) override;

            void addModule(std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>& module);
        };
      }
    }
  }
}
#endif