#include "driftless/hal/Coprocessor.hpp"

namespace driftless::hal {
void Coprocessor::taskLoop(void* params) {
  Coprocessor* coprocessor = static_cast<Coprocessor*>(params);
  while (true) {
    coprocessor->taskUpdate();
    coprocessor->m_delayer->delay(Coprocessor::TASK_DELAY);
  }
}

void Coprocessor::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  fetchLatestSignal();

  if (m_mutex) {
    m_mutex->give();
  }
}

void Coprocessor::fetchLatestSignal() {
  if (m_serial_device) {
    while (m_serial_device->getInputBytes()) {
      m_serial_buffer += static_cast<char>(m_serial_device->readByte());
    }
  }
}
Coprocessor::Coprocessor(std::unique_ptr<io::ISerialDevice>& serial_device)
    : m_serial_device{std::move(serial_device)} {}
}  // namespace driftless::hal