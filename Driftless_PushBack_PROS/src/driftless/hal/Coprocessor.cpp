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
  processLatestSignal();

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

void Coprocessor::processLatestSignal() {
  while (hasPacket()) {
    m_serial_buffer = m_serial_buffer.substr(
        m_serial_buffer.find(m_serial_protocol->getStartDelimiter()));

    std::string packet{m_serial_buffer.substr(
        0, m_serial_buffer.find(m_serial_protocol->getEndDelimiter()) + 1)};

    std::pair<std::string, std::string> packet_info{
        m_serial_protocol->decode(packet)};

    m_latest_data[packet_info.first] = packet_info.second;

    m_serial_buffer = m_serial_buffer.substr(
        m_serial_buffer.find(m_serial_protocol->getEndDelimiter()) + 1);
  }
}

bool Coprocessor::hasPacket() const {
  return m_serial_buffer.find(m_serial_protocol->getStartDelimiter()) !=
             std::string::npos &&
         m_serial_buffer.find(m_serial_protocol->getEndDelimiter()) !=
             std::string::npos;
}

void Coprocessor::init() {
  if (m_serial_device) {
    m_serial_device->initialize();
  }
}

void Coprocessor::run() {
  if (m_task) {
    m_task->start(&Coprocessor::taskLoop, this);
  }
}

std::string Coprocessor::getValue(std::string& key) {
  std::string value{};

  if (m_latest_data.contains(key)) {
    value = m_latest_data[key];
  }

  return value;
}

void Coprocessor::sendValue(std::string& key, const std::string& value) {
  if (m_serial_device && m_serial_protocol) {
    std::string packet = m_serial_protocol->encode(key, value);
    m_serial_device->write(reinterpret_cast<uint8_t*>(packet.data()),
                           packet.size());
  }
}
}  // namespace driftless::hal