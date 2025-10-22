#include "driftless/config/BlueConfig.hpp"

namespace driftless::config {
std::string BlueConfig::getName() { return CONFIG_NAME; }

std::shared_ptr<control::ControlSystem> BlueConfig::buildControlSystem() {
  std::shared_ptr<control::ControlSystem> control_system =
      std::make_shared<control::ControlSystem>();

  return control_system;
}

std::shared_ptr<processes::ProcessSystem> BlueConfig::buildProcessSystem() {
  std::shared_ptr<processes::ProcessSystem> process_system =
      std::make_shared<processes::ProcessSystem>();

  return process_system;
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
  std::unique_ptr<pros::Motor> pros_drive_front_left_top_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_LEFT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_front_left_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_LEFT_BOTTOM_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_front_right_top_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_RIGHT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_front_right_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_RIGHT_BOTTOM_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_left_top_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_LEFT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_left_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_LEFT_BOTTOM_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_right_top_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_RIGHT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_right_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_RIGHT_BOTTOM_PORT, DRIVE_GEARSET)};

  // adapt the pros objects
  std::unique_ptr<io::IMotor> drive_front_left_top_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_front_left_top_motor)};
  std::unique_ptr<io::IMotor> drive_front_left_bottom_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_front_left_bottom_motor)};
  std::unique_ptr<io::IMotor> drive_front_right_top_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_front_right_top_motor)};
  std::unique_ptr<io::IMotor> drive_front_right_bottom_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_front_right_bottom_motor)};
  std::unique_ptr<io::IMotor> drive_back_left_top_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_back_left_top_motor)};
  std::unique_ptr<io::IMotor> drive_back_left_bottom_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_back_left_bottom_motor)};
  std::unique_ptr<io::IMotor> drive_back_right_top_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_back_right_top_motor)};
  std::unique_ptr<io::IMotor> drive_back_right_bottom_motor{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_drive_back_right_bottom_motor)};

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
          drive_front_left_module_builder.withMotor(drive_front_left_top_motor)
              ->withMotor(drive_front_left_bottom_motor)
              ->withAngleOffset(DRIVE_FRONT_LEFT_ANGLE_OFFSET)
              ->build()};

  std::unique_ptr<robot::subsystems::holonomic_drive_train::
                      holonomic_drive_module::IHolonomicDriveModule>
      drive_front_right_module{
          drive_front_right_module_builder.withMotor(drive_front_right_top_motor)
              ->withMotor(drive_front_right_bottom_motor)
              ->withAngleOffset(DRIVE_FRONT_RIGHT_ANGLE_OFFSET)
              ->build()};

  std::unique_ptr<robot::subsystems::holonomic_drive_train::
                      holonomic_drive_module::IHolonomicDriveModule>
      drive_back_left_module{
          drive_back_left_module_builder.withMotor(drive_back_left_top_motor)
              ->withMotor(drive_back_left_bottom_motor)
              ->withAngleOffset(DRIVE_BACK_LEFT_ANGLE_OFFSET)
              ->build()};

  std::unique_ptr<robot::subsystems::holonomic_drive_train::
                      holonomic_drive_module::IHolonomicDriveModule>
      drive_back_right_module{
          drive_back_right_module_builder.withMotor(drive_back_right_top_motor)
              ->withMotor(drive_back_right_bottom_motor)
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

  // ## INTAKE SUBSYSTEM ##

  // create pros objects
  std::unique_ptr<pros::Motor> pros_intake_front_motor_1{
      std::make_unique<pros::Motor>(INTAKE_FRONT_MOTOR_1_PORT)};
  std::unique_ptr<pros::Motor> pros_intake_intermediary_motor_1{
      std::make_unique<pros::Motor>(INTAKE_INTERMEDIARY_MOTOR_1_PORT)};
  std::unique_ptr<pros::Motor> pros_intake_back_motor_1{
      std::make_unique<pros::Motor>(INTAKE_BACK_MOTOR_1_PORT)};
  std::unique_ptr<pros::adi::DigitalOut> pros_intake_back_arms{
      std::make_unique<pros::adi::DigitalOut>(INTAKE_BACK_ARMS_PORT)};
  std::unique_ptr<pros::Motor> pros_intake_vertical_motor_1{
      std::make_unique<pros::Motor>(INTAKE_VERTICAL_MOTOR_1_PORT)};

  // adapt the pros objects
  std::unique_ptr<io::IMotor> intake_front_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(pros_intake_front_motor_1)};
  std::unique_ptr<io::IMotor> intake_intermediary_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_intake_intermediary_motor_1)};
  std::unique_ptr<io::IMotor> intake_back_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(pros_intake_back_motor_1)};
  std::unique_ptr<io::IPiston> intake_back_arms{
      std::make_unique<pros_adapters::ProsPiston>(pros_intake_back_arms)};
  std::unique_ptr<io::IMotor> intake_vertical_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(
          pros_intake_vertical_motor_1)};

  // build the intake
  robot::subsystems::intake::DirectIntakeBuilder intake_builder{};

  std::unique_ptr<robot::subsystems::intake::IIntake> intake{
      intake_builder.withFrontMotor(intake_front_motor_1)
          ->withIntermediaryMotor(intake_intermediary_motor_1)
          ->withBackMotor(intake_back_motor_1)
          ->withVerticalMotor(intake_vertical_motor_1)
          ->withBackPiston(intake_back_arms)
          ->build()};

  // build the subsystem
  std::unique_ptr<robot::subsystems::ASubsystem> intake_subsystem{
      std::make_unique<robot::subsystems::intake::IntakeSubsystem>(intake)};

  // add subsystem to robot
  robot->addSubsystem(intake_subsystem);

  // ## HOOD SUBSYSTEM ##

  // create pros objects
  std::unique_ptr<pros::Motor> pros_hood_motor_1{
      std::make_unique<pros::Motor>(HOOD_MOTOR_1_PORT)};
  std::unique_ptr<pros::adi::DigitalOut> pros_hood_height_pistons{
      std::make_unique<pros::adi::DigitalOut>(HOOD_HEIGHT_PISTONS_PORT)};
  std::unique_ptr<pros::adi::DigitalOut> pros_hood_gate_pistons{
      std::make_unique<pros::adi::DigitalOut>(HOOD_GATE_PISTONS_PORT)};
  std::unique_ptr<pros::adi::DigitalOut> pros_hood_descore_pistons{
      std::make_unique<pros::adi::DigitalOut>(HOOD_DESCORE_PISTONS_PORT)};
    std::unique_ptr<pros::adi::DigitalOut> pros_hood_bump_pistons{
      std::make_unique<pros::adi::DigitalOut>(HOOD_BUMP_PISTONS_PORT)};

  // adapt the pros objects
  std::unique_ptr<io::IMotor> hood_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(pros_hood_motor_1)};
  std::unique_ptr<io::IPiston> hood_height_pistons{
      std::make_unique<pros_adapters::ProsPiston>(pros_hood_height_pistons)};
  std::unique_ptr<io::IPiston> hood_gate_pistons{
      std::make_unique<pros_adapters::ProsPiston>(pros_hood_gate_pistons)};
  std::unique_ptr<io::IPiston> hood_descore_pistons{
      std::make_unique<pros_adapters::ProsPiston>(pros_hood_descore_pistons)};
    std::unique_ptr<io::IPiston> hood_bump_pistons{
      std::make_unique<pros_adapters::ProsPiston>(pros_hood_bump_pistons)};

  // build the hood
  robot::subsystems::hood::DirectHoodBuilder hood_builder{};

  std::unique_ptr<robot::subsystems::hood::IHood> hood{
      hood_builder.withMotor(hood_motor_1)
          ->withHeightPiston(hood_height_pistons)
          ->withGatePiston(hood_gate_pistons)
          ->withDescorePiston(hood_descore_pistons)
          ->withBumpPiston(hood_bump_pistons)
          ->build()};

  // build the subsystem
  std::unique_ptr<robot::subsystems::ASubsystem> hood_subsystem{
      std::make_unique<robot::subsystems::hood::HoodSubsystem>(hood)};

  // add subsystem to robot
  robot->addSubsystem(hood_subsystem);

  // return complete robot
  return robot;
}
}  // namespace driftless::config