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
    serial_protocol::Packet packet{m_serial_protocol.decode(m_serial_buffer)};
    m_serial_buffer.erase(
        0, packet.value.size() +
               3);  // size of the value, plus 1 byte for start delimiter, one
                    // for packet size, and one for the key

    m_latest_data[packet.key] = packet.value;
  }
}

bool Coprocessor::hasPacket() const {
  return m_serial_protocol.hasValidPacket(m_serial_buffer);
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

template <typename T>
T Coprocessor::getValue(serial_protocol::ESerialKey key) {
  T value{};

  if (m_latest_data.contains(key)) {
    try {
      std::memcpy(&value, m_latest_data[key].data(),
                  sizeof(T));  // Copy the bytes from the string to the value
    } catch (const std::exception& e) {
      // Handle conversion error, e.g., log it or throw an exception
      // For now, we will just return the default value
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 10,
                          "Invalid conversion for %d at %7.2f", static_cast<int>(key), m_clock->getTime() / 1000.0);
      value = T{};
    }
  }

  return value;
}

template <typename T>
void Coprocessor::sendValue(serial_protocol::ESerialKey key, const T& value) {
  if (m_serial_device && m_serial_protocol) {
    std::string packet_info = m_serial_protocol->encode<T>(key, value);
    m_serial_device->write(reinterpret_cast<uint8_t*>(packet_info.data()),
                           packet.size());
  }
}

void Coprocessor::setSerialDevice(
    std::unique_ptr<io::ISerialDevice>& serial_device) {
  m_serial_device = std::move(serial_device);
}

void Coprocessor::setSerialProtocol(
    serial_protocol::SerialProtocol& serial_protocol) {
  m_serial_protocol = serial_protocol;
}

void Coprocessor::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}
void Coprocessor::setClock(std::unique_ptr<rtos::IClock>& clock) {
  m_clock = std::move(clock);
}

void Coprocessor::setDelayer(std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void Coprocessor::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}
}  // namespace driftless::hal