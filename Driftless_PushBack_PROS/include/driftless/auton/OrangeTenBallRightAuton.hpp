#ifndef __ORANGE_TEN_BALL_RIGHT_AUTON_HPP__
#define __ORANGE_TEN_BALL_RIGHT_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the orange robot's pump
/// and dump match auton
/// @author Matthew Backman
class OrangeTenBallRightAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 4.0};

  control::Point match_load_lineup{117.0, 24.0, M_PI / 2.0};

  control::Point match_load{117.0, 11.0, M_PI / 2.0};

  control::Point long_goal{116.0, 42.0, M_PI / 2.0};

  control::Point descore_lineup{128.0, 33.0, M_PI / 2.0};

  control::Point end_descore{128.0, 46.0, M_PI / 2.0};

 public:
  /// @brief Constructs a new orange pump n dump auton object
  OrangeTenBallRightAuton();

  /// @brief Initializes the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  void init(std::shared_ptr<driftless::robot::Robot>& robot,
            std::shared_ptr<driftless::control::ControlSystem>& control_system)
      override;

  /// @brief Runs the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The current
  /// alliance
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The system clock
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer used
  void run(std::shared_ptr<driftless::robot::Robot>& robot,
           std::shared_ptr<driftless::control::ControlSystem>& control_system,
           std::shared_ptr<driftless::alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer>& delayer) override;
};
}  // namespace auton
}  // namespace driftless
#endif