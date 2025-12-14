#include "driftless/hal/CoprocessorBuilder.hpp"

namespace driftless::hal {
CoprocessorBuilder* CoprocessorBuilder::withSerialDevice(
    std::unique_ptr<io::ISerialDevice>& serial_device) {
  m_serial_device = std::move(serial_device);
  return this;
}

CoprocessorBuilder* CoprocessorBuilder::withClock(
    std::unique_ptr<rtos::IClock>& clock) {
  m_clock = std::move(clock);
  return this;
}

CoprocessorBuilder* CoprocessorBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return this;
}

CoprocessorBuilder* CoprocessorBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

CoprocessorBuilder* CoprocessorBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

std::shared_ptr<Coprocessor> CoprocessorBuilder::build() {
  std::shared_ptr<Coprocessor> coprocessor{
      std::make_shared<Coprocessor>()};
  coprocessor->setSerialDevice(m_serial_device);
  coprocessor->setClock(m_clock);
  coprocessor->setDelayer(m_delayer);
  coprocessor->setMutex(m_mutex);
  coprocessor->setTask(m_task);
  return coprocessor;
}
}  // namespace driftless::hal