#include "driftless/robot/subsystems/intake/StateMachineIntakeBuilder.hpp"

namespace driftless::robot::subsystems::intake {
StateMachineIntakeBuilder* StateMachineIntakeBuilder::withFrontMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_front_motors.addMotor(motor);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withIntermediaryMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_intermediary_motors.addMotor(motor);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withBackMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_back_motors.addMotor(motor);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withVerticalMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_vertical_motors.addMotor(motor);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withBackPiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_back_pistons.addPiston(piston);
  return this;
}

StateMachineIntakeBuilder*
StateMachineIntakeBuilder::withFrontColorSensorDistance(double distance) {
  m_front_sensor_distance = distance;
  return this;
}

StateMachineIntakeBuilder*
StateMachineIntakeBuilder::withMidColorSensorDistance(double distance) {
  m_mid_sensor_distance = distance;
  return this;
}

StateMachineIntakeBuilder*
StateMachineIntakeBuilder::withBackColorSensorDistance(double distance) {
  m_back_sensor_distance = distance;
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withFrontColorSensor(
    std::unique_ptr<io::IColorSensor>& color_sensor) {
  m_front_color_sensor = std::move(color_sensor);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withMidColorSensor(
    std::unique_ptr<io::IColorSensor>& color_sensor) {
  m_mid_color_sensor = std::move(color_sensor);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withBackColorSensor(
    std::unique_ptr<io::IColorSensor>& color_sensor) {
  m_back_color_sensor = std::move(color_sensor);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

StateMachineIntakeBuilder* StateMachineIntakeBuilder::withState(
    intake_states::EIntakeStates state_name,
    std::unique_ptr<intake_states::IIntakeState>& state) {
  m_states[static_cast<uint8_t>(state_name)] = std::move(state);
  return this;
}

std::unique_ptr<IIntake> StateMachineIntakeBuilder::build() {
  std::unique_ptr<StateMachineIntake> state_machine_intake{
      std::make_unique<StateMachineIntake>()};
  state_machine_intake->setFrontMotors(m_front_motors);
  state_machine_intake->setIntermediaryMotors(m_intermediary_motors);
  state_machine_intake->setBackMotors(m_back_motors);
  state_machine_intake->setVerticalMotors(m_vertical_motors);
  state_machine_intake->setBackPistons(m_back_pistons);
  state_machine_intake->setFrontColorSensor(m_front_color_sensor);
  state_machine_intake->setMidColorSensor(m_mid_color_sensor);
  state_machine_intake->setBackColorSensor(m_back_color_sensor);
  state_machine_intake->setDelayer(m_delayer);
  state_machine_intake->setMutex(m_mutex);
  state_machine_intake->setTask(m_task);
  state_machine_intake->setFrontColorSensorDistanceToEnd(
      m_front_sensor_distance);
  state_machine_intake->setMidColorSensorDistanceToEnd(m_mid_sensor_distance);
  state_machine_intake->setBackColorSensorDistanceToEnd(m_back_sensor_distance);
  state_machine_intake->setStates(m_states);

  return state_machine_intake;
}
}  // namespace driftless::robot::subsystems::intake