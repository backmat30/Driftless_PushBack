#include "driftless/config/BlueConfig.hpp"

namespace driftless::config {
std::string BlueConfig::getName() { return CONFIG_NAME; }

std::shared_ptr<control::ControlSystem> BlueConfig::buildControlSystem() {
  std::shared_ptr<control::ControlSystem> control_system =
      std::make_shared<control::ControlSystem>();

  // clock and delayer for controls

  std::unique_ptr<rtos::IClock> clock{
      std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};

  // ## TRAJECTORY FOLLOWER ##

  // rtos
  std::unique_ptr<rtos::ITask> trajectory_follower_task{
      std::make_unique<pros_adapters::ProsTask>()};
  std::unique_ptr<rtos::IMutex> trajectory_follower_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};

  // PID controllers
  control::PID trajectory_follower_x_pid{clock, TRAJECTORY_FOLLOWER_X_KP,
                                         TRAJECTORY_FOLLOWER_X_KI,
                                         TRAJECTORY_FOLLOWER_X_KD};
  control::PID trajectory_follower_y_pid{clock, TRAJECTORY_FOLLOWER_Y_KP,
                                         TRAJECTORY_FOLLOWER_Y_KI,
                                         TRAJECTORY_FOLLOWER_Y_KD};
  control::PID trajectory_follower_theta_PID{
      clock, TRAJECTORY_FOLLOWER_THETA_KP, TRAJECTORY_FOLLOWER_THETA_KI,
      TRAJECTORY_FOLLOWER_THETA_KD};

  // build the trajectory follower
  control::trajectory::trajectory_follower::PIDTrajectoryFollowerBuilder
      trajectory_follower_builder{};

  std::unique_ptr<control::trajectory::trajectory_follower::ITrajectoryFollower>
      trajectory_follower{
          trajectory_follower_builder.withClock(clock)
              ->withDelayer(delayer)
              ->withTask(trajectory_follower_task)
              ->withMutex(trajectory_follower_mutex)
              ->withXPID(trajectory_follower_x_pid)
              ->withYPID(trajectory_follower_y_pid)
              ->withThetaPID(trajectory_follower_theta_PID)
              ->withTargetTolerance(TRAJECTORY_FOLLOWER_TARGET_TOLERANCE)
              ->withTargetVelocity(TRAJECTORY_FOLLOWER_TARGET_VELOCITY)
              ->build()};

  std::unique_ptr<control::AControl> trajectory_follower_control{
      std::make_unique<
          control::trajectory::trajectory_follower::TrajectoryFollowerControl>(
          trajectory_follower)};

  control_system->addControl(trajectory_follower_control);

  // ## MOTION CONTROL ##

  // rtos
  std::unique_ptr<rtos::ITask> drive_straight_task{
      std::make_unique<pros_adapters::ProsTask>()};
  std::unique_ptr<rtos::IMutex> drive_straight_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};

  std::unique_ptr<rtos::ITask> turn_task{
      std::make_unique<pros_adapters::ProsTask>()};
  std::unique_ptr<rtos::IMutex> turn_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};

  std::unique_ptr<rtos::ITask> go_to_point_task{
      std::make_unique<pros_adapters::ProsTask>()};
  std::unique_ptr<rtos::IMutex> go_to_point_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};

  // PID controllers
  control::PID drive_straight_linear_pid{clock, DRIVE_STRAIGHT_LINEAR_KP,
                                         DRIVE_STRAIGHT_LINEAR_KI,
                                         DRIVE_STRAIGHT_LINEAR_KD};
  control::PID drive_straight_angular_pid{clock, DRIVE_STRAIGHT_ANGULAR_KP,
                                          DRIVE_STRAIGHT_ANGULAR_KI,
                                          DRIVE_STRAIGHT_ANGULAR_KD};

  control::PID turn_pid{clock, TURN_KP, TURN_KI, TURN_KD};

  control::PID go_to_point_x_pid{clock, GO_TO_POINT_X_KP, GO_TO_POINT_X_KI,
                                 GO_TO_POINT_X_KD};
  control::PID go_to_point_y_pid{clock, GO_TO_POINT_Y_KP, GO_TO_POINT_Y_KI,
                                 GO_TO_POINT_Y_KD};
  control::PID go_to_point_rotational_pid{clock, GO_TO_POINT_ROTATIONAL_KP,
                                          GO_TO_POINT_ROTATIONAL_KI,
                                          GO_TO_POINT_ROTATIONAL_KD};

  // build the motion controls
  control::motion::PIDDriveStraightBuilder drive_straight_builder{};
  control::motion::PIDTurnBuilder turn_builder{};
  control::motion::PIDHolonomicGoToPointBuilder go_to_point_builder{};

  std::unique_ptr<control::motion::IDriveStraight> drive_straight{
      drive_straight_builder.withDelayer(delayer)
          ->withMutex(drive_straight_mutex)
          ->withTask(drive_straight_task)
          ->withLinearPID(drive_straight_linear_pid)
          ->withRotationalPID(drive_straight_angular_pid)
          ->withTargetTolerance(MOTION_LINEAR_DISTANCE_TOLERANCE)
          ->withTargetVelocity(1.0)
          ->build()};

  std::unique_ptr<control::motion::ITurn> turn{
      turn_builder.withDelayer(delayer)
          ->withMutex(turn_mutex)
          ->withTask(turn_task)
          ->withRotationalPID(turn_pid)
          ->withTargetTolerance(MOTION_ANGULAR_DISTANCE_TOLERANCE)
          ->withTargetVelocity(0.1)
          ->build()};

  std::unique_ptr<control::motion::IGoToPoint> go_to_point{
      go_to_point_builder.withDelayer(delayer)
          ->withMutex(go_to_point_mutex)
          ->withTask(go_to_point_task)
          ->withXPID(go_to_point_x_pid)
          ->withYPID(go_to_point_y_pid)
          ->withRotationalPID(go_to_point_rotational_pid)
          ->withVelocityTolerance(MOTION_LINEAR_VELOCITY_TOLERANCE)
          ->withDistanceTolerance(MOTION_LINEAR_DISTANCE_TOLERANCE)
          ->withAngularTolerance(MOTION_ANGULAR_DISTANCE_TOLERANCE)
          ->build()};

  // make the controller
  std::unique_ptr<control::AControl> motion_control{
      std::make_unique<control::motion::MotionControl>(drive_straight,
                                                       go_to_point, turn)};

  // add to the control system
  control_system->addControl(motion_control);

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

  // ## ARDUINO ##
  // create pros objects
  std::unique_ptr<pros::Serial> pros_arduino_serial{
      std::make_unique<pros::Serial>(ARDUINO_PORT, 115200)};

  // adapt the pros objects
  std::unique_ptr<io::ISerialDevice> arduino_serial{
      std::make_unique<pros_adapters::ProsSerialDevice>(pros_arduino_serial)};
  std::unique_ptr<rtos::IClock> arduino_clock{
      std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> arduino_delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};
  std::unique_ptr<rtos::IMutex> arduino_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};
  std::unique_ptr<rtos::ITask> arduino_task{
      std::make_unique<pros_adapters::ProsTask>()};

  // create the arduino coprocessor
  hal::CoprocessorBuilder arduino_builder;

  std::shared_ptr<hal::Coprocessor> arduino_coprocessor{
      arduino_builder.withSerialDevice(arduino_serial)
          ->withClock(arduino_clock)
          ->withDelayer(arduino_delayer)
          ->withMutex(arduino_mutex)
          ->withTask(arduino_task)
          ->build()};

  // ## DRIVE TRAIN ##

  // create pros objects
  std::unique_ptr<pros::Motor> pros_drive_front_left_top_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_LEFT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_front_left_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_LEFT_BOTTOM_PORT,
                                    DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_front_right_top_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_RIGHT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_front_right_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_FRONT_RIGHT_BOTTOM_PORT,
                                    DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_left_top_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_LEFT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_left_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_LEFT_BOTTOM_PORT,
                                    DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_right_top_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_RIGHT_TOP_PORT, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> pros_drive_back_right_bottom_motor{
      std::make_unique<pros::Motor>(DRIVE_BACK_RIGHT_BOTTOM_PORT,
                                    DRIVE_GEARSET)};

  // rtos
  std::unique_ptr<rtos::IDelayer> drive_delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};
  std::unique_ptr<rtos::ITask> drive_task{
      std::make_unique<pros_adapters::ProsTask>()};
  std::unique_ptr<rtos::IMutex> drive_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};

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
          drive_front_right_module_builder
              .withMotor(drive_front_right_top_motor)
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
                      ->withDelayer(drive_delayer)
                      ->withTask(drive_task)
                      ->withMutex(drive_mutex)
                      ->withMaxLinearVelocity(DRIVE_MAX_LINEAR_VELOCITY)
                      ->withMaxAngularVelocity(DRIVE_MAX_ANGULAR_VELOCITY)
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

  // ## ODOMETRY SUBSYSTEM ##

  // create pros adapters
  std::unique_ptr<rtos::IClock> odom_clock{
      std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> odom_delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};
  std::unique_ptr<rtos::IMutex> odom_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};
  std::unique_ptr<rtos::ITask> odom_task{
      std::make_unique<pros_adapters::ProsTask>()};

  // build the arduino position tracker
  robot::subsystems::odometry::SparkFunPositionTrackerBuilder
      position_tracker_builder{};

  std::unique_ptr<robot::subsystems::odometry::IPositionTracker>
      position_tracker{position_tracker_builder.withClock(odom_clock)
                           ->withDelayer(odom_delayer)
                           ->withMutex(odom_mutex)
                           ->withTask(odom_task)
                           ->withCoprocessor(arduino_coprocessor)
                           ->withLocalXOffset(ODOMETRY_LOCAL_X_OFFSET)
                           ->withLocalYOffset(ODOMETRY_LOCAL_Y_OFFSET)
                           ->withLocalThetaOffset(ODOMETRY_LOCAL_THETA_OFFSET)
                           ->build()};

  std::unique_ptr<robot::subsystems::ASubsystem> odometry_subsystem{
      std::make_unique<robot::subsystems::odometry::OdometrySubsystem>(
          position_tracker)};

  // add subsystem to robot
  robot->addSubsystem(odometry_subsystem);

  // ## BRAKE SUBSYSTEM ##

  // create pros objects
  std::unique_ptr<pros::adi::DigitalOut> pros_brake_piston{
      std::make_unique<pros::adi::DigitalOut>(BRAKE_PISTON_PORT)};

  // adapt pros objects
  std::unique_ptr<io::IPiston> adapted_brake_piston{
      std::make_unique<pros_adapters::ProsPiston>(pros_brake_piston)};

  // build the brake
  robot::subsystems::brake::PneumaticBrakeBuilder brake_builder{};

  std::unique_ptr<robot::subsystems::brake::IBrake> brake{
      brake_builder.withBrakePiston(adapted_brake_piston)->build()};

  // create the subsystem
  std::unique_ptr<robot::subsystems::ASubsystem> brake_subsystem{
      std::make_unique<robot::subsystems::brake::BrakeSubsystem>(brake)};

  robot->addSubsystem(brake_subsystem);

  // return complete robot
  return robot;
}
}  // namespace driftless::config