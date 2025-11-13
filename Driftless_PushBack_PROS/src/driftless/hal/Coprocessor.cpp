#include "driftless/hal/Coprocessor.hpp"

#include <iostream>
namespace driftless::hal {
void Coprocessor::taskLoop(void* params) {
  Coprocessor* coprocessor = static_cast<Coprocessor*>(params);
  while (true) {
    coprocessor->taskUpdate();
  }
}

void Coprocessor::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  uint32_t start_time{m_clock->getTime()};

  fetchLatestSignal();
  processLatestSignal();
  sendOutgoingPackage();

  uint32_t elapsed_time{m_clock->getTime() - start_time};
  m_delayer->delay(Coprocessor::TASK_DELAY - elapsed_time);

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
  m_serial_buffer.clear();
  if (m_serial_device) {
    if (m_serial_device->getInputBytes() >= 2) {
      uint64_t fetch_start_time{m_clock->getTime()};

      // read until start delimiter is found
      while (m_serial_device->getInputBytes() &&
             m_serial_device->readByte() != 0xFF &&
             fetch_start_time + 100 > m_clock->getTime());

      if (m_serial_device->getInputBytes()) {
        m_serial_buffer.push_back(0xFF);
        m_serial_buffer.push_back(m_serial_device->readByte());
      }
      uint8_t package_size{m_serial_buffer[1]};
      while (package_size > m_serial_buffer.size() &&
             fetch_start_time + 100 > m_clock->getTime()) {
        if (m_serial_device->getInputBytes()) {
          m_serial_buffer.push_back(m_serial_device->readByte());
        }
      }
      while (!isValidSignal() && fetch_start_time + 100 > m_clock->getTime()) {
        m_serial_buffer.erase(m_serial_buffer.begin());
        while (m_serial_buffer.size() > 1 && m_serial_buffer[0] != 0xFF &&
               fetch_start_time + 100 > m_clock->getTime()) {
          m_serial_buffer.erase(m_serial_buffer.begin());
        }

        if (m_serial_buffer.size() < 1) {
          package_size = m_serial_buffer[1];
          while (package_size > m_serial_buffer.size() &&
                 fetch_start_time + 100 > m_clock->getTime()) {
            if (m_serial_device->getInputBytes()) {
              m_serial_buffer.push_back(m_serial_device->readByte());
            }
          }
        }
        for (uint8_t& byte : m_serial_buffer) {
          std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
      }
      std::cout << "Final Buffer: ";
      for (uint8_t& byte : m_serial_buffer) {
        std::cout << std::hex << static_cast<int>(byte) << " ";
      }
      std::cout << std::endl;
    }
  }
}

void Coprocessor::processLatestSignal() {
  if (m_serial_buffer.size() < 2) {
    std::cout << "Buffer too small" << std::endl;
    return;
  }
  if (m_serial_buffer.size() != m_serial_buffer[1]) {
    std::cout << "Size mismatch" << std::endl;
    return;
  }
  if (!isValidSignal()) {
    std::cout << "Invalid CRC" << std::endl;
    return;
  }

  uint8_t packet_count{m_serial_buffer[2]};
  uint8_t index{3};

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
  std::cout << "Recieved CRC: " << std::hex << recieved_crc << std::endl;
  uint16_t calculated_crc =
      calculateCRC(m_serial_buffer, m_serial_buffer.size() - 2);
  std::cout << "Calculated CRC: " << std::hex << calculated_crc << std::endl;
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

  if (serialized_data[2] != 0) {
    uint16_t crc{calculateCRC(serialized_data, serialized_data.size())};
    uint8_t* crc_bytes = reinterpret_cast<uint8_t*>(&crc);
    serialized_data.insert(serialized_data.end(), crc_bytes, crc_bytes + 2);

    if (m_serial_device) {
      m_serial_device->write(serialized_data.data(),
                             static_cast<int>(serialized_data.size()));
      for (uint8_t& byte : serialized_data) {
        std::cout << std::hex << static_cast<int>(byte) << " ";
      }
      std::cout << std::endl;
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