#ifndef __MODULAR_HOLONOMIC_DRIVE_HPP__
#define __MODULAR_HOLONOMIC_DRIVE_HPP__

#include <memory>
#include <vector>

#include "driftless/robot/subsystems/holonomic_drive_train/IHolonomicDrive.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/IHolonomicDriveModule.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

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

/// @brief Class representing a modular holonomic drive train
/// @author Matthew Backman
class ModularHolonomicDrive : public IHolonomicDrive {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly updates the modular holonomic drive
  /// @param params __void*__ The modular holonomic drive to update
  static void taskLoop(void* params);

  /// @brief The modules controlled by the drive train
  std::vector<std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>>
      m_modules{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  double m_max_linear_velocity{};

  double m_max_angular_velocity{};

  HolonomicMotionVector m_current_velocity{};

  void taskUpdate();

 public:
  /// @brief Initializes the modular holonomic drive
  void init() override;

  /// @brief Runs the modular holonomic drive
  void run() override;

  /// @brief Sets the motion vector of the modular holonomic drive
  /// @param motion_vector __HolonomicMotionVector__ Vector representing the
  /// desired motion of the robot (robot-centric)
  void setMotionVector(HolonomicMotionVector motion_vector) override;

  /// @brief Sets the X velocity of the modular holonomic drive
  /// @param x_velocity The desired X velocity
  void setXVelocity(double x_velocity);

  /// @brief Sets the Y velocity of the modular holonomic drive
  /// @param y_velocity The desired Y velocity
  void setYVelocity(double y_velocity);

  /// @brief Sets the angular velocity of the modular holonomic drive
  /// @param angular_velocity The desired angular velocity
  void setAngularVelocity(double angular_velocity);

  /// @brief Sets the normalized motion vector of the modular holonomic drive
  /// @param motion_vector Vector representing the desired normalized motion of the robot (robot-centric)
  void setNormalizedMotionVector(HolonomicMotionVector motion_vector) override;

    /// @brief Sets the normalized X velocity of the modular holonomic drive
    /// @param x_velocity The desired normalized X velocity [-1, 1]
  void setNormalizedXVelocity(double x_velocity);

  /// @brief Sets the normalized Y velocity of the modular holonomic drive
  /// @param y_velocity The desired normalized Y velocity [-1, 1]
  void setNormalizedYVelocity(double y_velocity);

  /// @brief Sets the normalized angular velocity of the modular holonomic drive
  /// @param angular_velocity The desired normalized angular velocity [-1, 1]
  void setNormalizedAngularVelocity(double angular_velocity);

  /// @brief Adds a module to the modular holonomic drive
  /// @param module __unique_ptr<IHolonomicDriveModule>&__ The module to be
  /// added
  void setModules(std::vector<std::unique_ptr<
                      holonomic_drive_module::IHolonomicDriveModule>>& modules);


};
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif