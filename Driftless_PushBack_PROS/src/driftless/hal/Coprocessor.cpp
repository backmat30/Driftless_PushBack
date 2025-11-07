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
  sendOutgoingPackage();

  if (m_mutex) {
    m_mutex->give();
  }
}

void Coprocessor::handleXposCommand(const uint8_t* data) {
  if (data) {
    std::vector<uint8_t> xpos_data(data, data + sizeof(float));

    m_latest_data[serial_protocol::ESerialKey::XPOS] = xpos_data;
  }
}

void Coprocessor::handleYposCommand(const uint8_t* data) {
  if (data) {
    std::vector<uint8_t> ypos_data(data, data + sizeof(float));

    m_latest_data[serial_protocol::ESerialKey::YPOS] = ypos_data;
  }
}

void Coprocessor::handleThetaCommand(const uint8_t* data) {
  if (data) {
    std::vector<uint8_t> theta_data(data, data + sizeof(float));

    m_latest_data[serial_protocol::ESerialKey::THETA] = theta_data;
  }
}

void Coprocessor::fetchLatestSignal() {
  if (m_serial_device) {
    if (m_serial_device->getInputBytes()) {
      uint64_t fetch_start_time{m_clock->getTime()};
      m_serial_device->read(m_serial_buffer.data(),
                            m_serial_device->getInputBytes());

      uint8_t package_size{m_serial_buffer[0]};
      while (package_size > m_serial_buffer.size() &&
             fetch_start_time + 100 > m_clock->getTime()) {
        if (m_serial_device->getInputBytes()) {
          m_serial_buffer.push_back(m_serial_device->readByte());
        }
      }
    }
  }
}

void Coprocessor::processLatestSignal() {
  if (m_serial_buffer.size() != m_serial_buffer[0]) {
    return;
  }
  if (!isValidSignal()) {
    return;
  }

  uint8_t packet_count{m_serial_buffer[1]};
  uint8_t index{2};

  for (uint8_t i = 0; i < packet_count; ++i) {
    serial_protocol::ESerialKey key =
        static_cast<serial_protocol::ESerialKey>(m_serial_buffer[index++]);
    uint8_t data_size{};
    if (m_command_sizes.contains(key)) {
      data_size = m_command_sizes.at(key);
    } else {
      return;
    }
    if (m_command_handlers.contains(key)) {
      (this->*m_command_handlers.at(key))(&m_serial_buffer[index]);
    }
    index += data_size;
  }

  m_serial_buffer.clear();
}

bool Coprocessor::isValidSignal() const {
  uint16_t recieved_crc{};
  if (m_serial_buffer.size() > 2) {
    std::memcpy(&recieved_crc,
                m_serial_buffer.data() + m_serial_buffer.size() - 2, 2);
  }
  uint16_t calculated_crc =
      calculateCRC(m_serial_buffer, m_serial_buffer.size() - 2);
  return recieved_crc == calculated_crc;
}

uint16_t Coprocessor::calculateCRC(const std::vector<uint8_t>& data,
                                   uint8_t size) const {
  uint16_t crc{0xFFFF};
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint8_t>(data[i]);
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void Coprocessor::sendOutgoingPackage() {
  for (const auto& packet : m_recurring_packets) {
    m_outgoing_package.addPacket(packet);
  }

  std::vector<uint8_t> serialized_data{
      m_outgoing_package.getSerializedPackage()};

  if (serialized_data[1] != 0) {
    uint16_t crc{calculateCRC(serialized_data, serialized_data.size())};
    uint8_t* crc_bytes = reinterpret_cast<uint8_t*>(&crc);
    serialized_data.insert(serialized_data.end(), crc_bytes, crc_bytes + 2);

    if (m_serial_device) {
      m_serial_device->write(serialized_data.data(),
                             static_cast<int>(serialized_data.size()));
    }
  }
  m_outgoing_package.clearPackets();
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
                          "Invalid conversion for %d at %7.2f",
                          static_cast<char>(key), m_clock->getTime() / 1000.0);
      value = T{};
    }
  }

  return value;
}

template <typename T>
void Coprocessor::addPacketToPackage(serial_protocol::ESerialKey key,
                                     const T& value) {
  std::vector<uint8_t> byte_vector(sizeof(T));
  std::memcpy(byte_vector.data(), &value, sizeof(T));
  m_outgoing_package.addPacket(key, byte_vector);
}

template <typename T>
void Coprocessor::addRecurringPacket(serial_protocol::ESerialKey key,
                                     const T& value) {
  std::vector<uint8_t> byte_vector(sizeof(T));
  std::memcpy(byte_vector.data(), &value, sizeof(T));
  serial_protocol::Packet packet{key, byte_vector};

  m_recurring_packets.push_back(packet);
}

template <typename T>
void Coprocessor::removeRecurringPacket(serial_protocol::ESerialKey key,
                                        const T& value) {
  std::vector<uint8_t> byte_vector(sizeof(T));
  std::memcpy(byte_vector.data(), &value, sizeof(T));
  serial_protocol::Packet packet{key, byte_vector};

  for(int i = 0; i < m_recurring_packets.size(); ++i) {
    if (m_recurring_packets[i] == packet) {
      m_recurring_packets.erase(m_recurring_packets.begin() + i);
      break;
    }
  }
}

void Coprocessor::setSerialDevice(
    std::unique_ptr<io::ISerialDevice>& serial_device) {
  m_serial_device = std::move(serial_device);
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