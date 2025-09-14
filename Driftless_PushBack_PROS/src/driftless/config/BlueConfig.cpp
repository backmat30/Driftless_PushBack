#include "driftless/config/BlueConfig.hpp"

namespace driftless::config {
std::string BlueConfig::getName() { return CONFIG_NAME; }

std::shared_ptr<control::ControlSystem> BlueConfig::buildControlSystem() {
  std::shared_ptr<control::ControlSystem> control_system =
      std::make_shared<control::ControlSystem>();

  return control_system;
}

std::shared_ptr<io::IController> BlueConfig::buildController() {
  // creates a default pros controller
  std::unique_ptr<pros::Controller> pros_controller{
      std::make_unique<pros::Controller>(pros::E_CONTROLLER_MASTER)};
  // adapts the pros controller to work as an IController object
  std::shared_ptr<io::IController> adapted_controller{
      std::make_shared<pros_adapters::ProsController>(pros_controller)};
  // send back a new adapted controller
  return adapted_controller;
}

std::shared_ptr<robot::Robot> BlueConfig::buildRobot() {
  std::shared_ptr<robot::Robot> robot{std::make_shared<robot::Robot>()};

  // ## DRIVE TRAIN ##

  // create pros objects
  std::unique_ptr<pros::Motor> pros_drive_front_left_motor_1{
      std::make_unique<pros::Motor>(DRIVE_FRONT_LEFT_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_front_right_motor_1{
      std::make_unique<pros::Motor>(DRIVE_FRONT_RIGHT_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_left_motor_1{
      std::make_unique<pros::Motor>(DRIVE_BACK_LEFT_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_right_motor_1{
      std::make_unique<pros::Motor>(DRIVE_BACK_RIGHT_PORT, DRIVE_GEARSET)};

  // adapt the pros objects
  std::unique_ptr<io::IMotor> drive_front_left_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_front_left_motor_1)};
  std::unique_ptr<io::IMotor> drive_front_right_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_front_right_motor_1)};
  std::unique_ptr<io::IMotor> drive_back_left_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_back_left_motor_1)};
  std::unique_ptr<io::IMotor> drive_back_right_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_back_right_motor_1)};

  // create the drive module builders
  robot::subsystems::holonomic_drive_train::holonomic_drive_module::
      XDriveModuleBuilder drive_front_left_module_builder;
  robot::subsystems::holonomic_drive_train::holonomic_drive_module::
      XDriveModuleBuilder drive_front_right_module_builder;
  robot::subsystems::holonomic_drive_train::holonomic_drive_module::
      XDriveModuleBuilder drive_back_left_module_builder;
  robot::subsystems::holonomic_drive_train::holonomic_drive_module::
      XDriveModuleBuilder drive_back_right_module_builder;

  // build the drive modules
  std::unique_ptr<robot::subsystems::holonomic_drive_train::
                      holonomic_drive_module::IHolonomicDriveModule>
      drive_front_left_module{
          drive_front_left_module_builder.withMotor(drive_front_left_motor_1)
              ->withAngleOffset(DRIVE_FRONT_LEFT_ANGLE_OFFSET)
              ->build()};

  std::unique_ptr<robot::subsystems::holonomic_drive_train::
                      holonomic_drive_module::IHolonomicDriveModule>
      drive_front_right_module{
          drive_front_right_module_builder.withMotor(drive_front_right_motor_1)
              ->withAngleOffset(DRIVE_FRONT_RIGHT_ANGLE_OFFSET)
              ->build()};

  std::unique_ptr<robot::subsystems::holonomic_drive_train::
                      holonomic_drive_module::IHolonomicDriveModule>
      drive_back_left_module{
          drive_back_left_module_builder.withMotor(drive_back_left_motor_1)
              ->withAngleOffset(DRIVE_BACK_LEFT_ANGLE_OFFSET)
              ->build()};

  std::unique_ptr<robot::subsystems::holonomic_drive_train::
                      holonomic_drive_module::IHolonomicDriveModule>
      drive_back_right_module{
          drive_back_right_module_builder.withMotor(drive_back_right_motor_1)
              ->withAngleOffset(DRIVE_BACK_RIGHT_ANGLE_OFFSET)
              ->build()};

  // build the drive train
  robot::subsystems::holonomic_drive_train::ModularHolonomicDriveBuilder
      drive_train_builder;

  std::unique_ptr<robot::subsystems::holonomic_drive_train::IHolonomicDrive>
      drive_train{drive_train_builder.withModule(drive_front_left_module)
                      ->withModule(drive_front_right_module)
                      ->withModule(drive_back_left_module)
                      ->withModule(drive_back_right_module)
                      ->build()};

  // create the subsystem
  std::unique_ptr<robot::subsystems::ASubsystem> drive_train_subsystem{
      std::make_unique<robot::subsystems::holonomic_drive_train::
                           HolonomicDriveTrainSubsystem>(drive_train)};

  // add the subsystem to the robot
  robot->addSubsystem(drive_train_subsystem);

  return robot;
}
}  // namespace driftless::config