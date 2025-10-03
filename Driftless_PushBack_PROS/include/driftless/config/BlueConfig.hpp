#ifndef __BLUE_CONFIG_HPP__
#define __BLUE_CONFIG_HPP__

#include <memory>

// config interface
#include "driftless/config/IConfig.hpp"

// pros objects
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// hardware interface includes
#include "driftless/hal/SparkfunOTOS.hpp"
#include "driftless/hal/TrackingWheel.hpp"
#include "driftless/io/IColorSensor.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/io/IDistanceSensor.hpp"
#include "driftless/io/IDistanceTracker.hpp"
#include "driftless/io/IInertialSensor.hpp"
#include "driftless/io/IMotor.hpp"
#include "driftless/io/IPiston.hpp"
#include "driftless/io/IPotentiometer.hpp"
#include "driftless/io/IRotationSensor.hpp"

// rtos includes
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

// pros adapter includes
#include "driftless/pros_adapters/ProsADIPotentiometer.hpp"
#include "driftless/pros_adapters/ProsClock.hpp"
#include "driftless/pros_adapters/ProsColorSensor.hpp"
#include "driftless/pros_adapters/ProsController.hpp"
#include "driftless/pros_adapters/ProsDelayer.hpp"
#include "driftless/pros_adapters/ProsDistanceSensor.hpp"
#include "driftless/pros_adapters/ProsInertialSensor.hpp"
#include "driftless/pros_adapters/ProsMutex.hpp"
#include "driftless/pros_adapters/ProsPiston.hpp"
#include "driftless/pros_adapters/ProsRotationSensor.hpp"
#include "driftless/pros_adapters/ProsSerialDevice.hpp"
#include "driftless/pros_adapters/ProsTask.hpp"
#include "driftless/pros_adapters/ProsV5Motor.hpp"

// control system includes
#include "driftless/control/ControlSystem.hpp"
#include "driftless/control/motion/MotionControl.hpp"
#include "driftless/control/motion/PIDDriveStraightBuilder.hpp"
#include "driftless/control/motion/PIDGoToPointBuilder.hpp"
#include "driftless/control/motion/PIDTurnBuilder.hpp"
#include "driftless/control/path/PIDPathFollowerBuilder.hpp"
#include "driftless/control/path/PathFollowerControl.hpp"

// robot include
#include "driftless/robot/Robot.hpp"

// holonomic drive subsystem includes
#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicDriveTrainSubsystem.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/ModularHolonomicDriveBuilder.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/XDriveModuleBuilder.hpp"

// intake includes
#include "driftless/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "driftless/robot/subsystems/intake/DirectIntakeBuilder.hpp"

// hood includes
#include "driftless/robot/subsystems/hood/HoodSubsystem.hpp"
#include "driftless/robot/subsystems/hood/DirectHoodBuilder.hpp"

namespace driftless {
namespace config {
class BlueConfig : public IConfig {
 private:
 static constexpr char CONFIG_NAME[] = "BLUE_CONFIG";

 // #### PORT NUMBERS ####
 
 // ## DRIVE MOTORS ##

 static constexpr int DRIVE_FRONT_LEFT_PORT{1};
 static constexpr int DRIVE_FRONT_RIGHT_PORT{4};
 static constexpr int DRIVE_BACK_LEFT_PORT{2};
 static constexpr int DRIVE_BACK_RIGHT_PORT{5};

 // ## INTAKE MOTORS ##

 static constexpr int INTAKE_FRONT_MOTOR_1_PORT{-6};
 static constexpr int INTAKE_INTERMEDIARY_MOTOR_1_PORT{-9};
 static constexpr int INTAKE_BACK_MOTOR_1_PORT{7};
 static constexpr int INTAKE_VERTICAL_MOTOR_1_PORT{8};

 // ## INTAKE PNEUMATICS ##

 static constexpr int INTAKE_BACK_ARMS_PORT{4};

 // ## HOOD MOTORS ##

 static constexpr int HOOD_MOTOR_1_PORT{-10};
 
 // ## HOOD PNEUMATICS ##

 static constexpr int HOOD_HEIGHT_PISTONS_PORT{2};
 static constexpr int HOOD_GATE_PISTONS_PORT{3};

 // #### ROBOT CONSTANTS ####

 // ## DRIVE ##

 static constexpr pros::MotorGearset DRIVE_GEARSET{pros::E_MOTOR_GEAR_BLUE};
 static constexpr double DRIVE_FRONT_LEFT_ANGLE_OFFSET{-M_PI / 4};
  static constexpr double DRIVE_FRONT_RIGHT_ANGLE_OFFSET{M_PI / 4}; 
 static constexpr double DRIVE_BACK_LEFT_ANGLE_OFFSET{3* -M_PI / 4};
 static constexpr double DRIVE_BACK_RIGHT_ANGLE_OFFSET{3 * M_PI / 4};

 public:
  std::string getName() override;
  std::shared_ptr<control::ControlSystem> buildControlSystem() override;
  std::shared_ptr<io::IController> buildController() override;
  std::shared_ptr<robot::Robot> buildRobot() override;
  std::shared_ptr<processes::ProcessSystem> buildProcessSystem() override;
};
}  // namespace config
}  // namespace driftless
#endif