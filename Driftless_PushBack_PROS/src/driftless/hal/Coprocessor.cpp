#include "driftless/hal/Coprocessor.hpp"

#include <iostream>
namespace driftless::hal {
void Coprocessor::taskLoop(void* params) {
  Coprocessor* coprocessor = static_cast<Coprocessor*>(params);
  while (true) {
    coprocessor->taskUpdate();
  }
}

uint16_t Coprocessor::calculateCRC(uint8_t* data, size_t size) {
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

void Coprocessor::encodeCOBS() {
  size_t read_index{};
  size_t write_index{1};
  size_t latest_zero_index{};
  uint8_t bytes_since_zero{1};

  std::array<uint8_t, 255> temp_buffer{};

  while (read_index < m_output_size) {
    if (m_output_buffer[read_index] == 0) {
      temp_buffer[latest_zero_index] = bytes_since_zero;
      bytes_since_zero = 1;
      latest_zero_index = write_index++;
      read_index++;

    } else {
      temp_buffer[write_index++] = m_output_buffer[read_index++];
      bytes_since_zero++;

      if (bytes_since_zero == 0xFF) {
        temp_buffer[latest_zero_index] = bytes_since_zero;
        bytes_since_zero = 1;
        latest_zero_index = write_index++;
      }
    }
  }

  m_output_size = write_index;
  temp_buffer[latest_zero_index] = bytes_since_zero;

  std::memcpy(m_output_buffer.data(), temp_buffer.data(), m_output_size);
}

bool Coprocessor::decodeCOBS() {
  size_t read_index = 0;
  size_t write_index = 0;

  while (read_index < m_bytes_read) {
    uint8_t code = m_input_package[read_index];

    if (code == 0) {
      return false;  // invalid in COBS stream
    }

    read_index++;

    // Copy (code - 1) bytes
    for (uint8_t i = 1; i < code; ++i) {
      if (read_index >= m_bytes_read) {
        return false;  // overflow
      }

      m_decoded_input_package[write_index++] = m_input_package[read_index++];
    }

    // Insert zero if needed
    if (code < 0xFF && read_index < m_bytes_read) {
      m_decoded_input_package[write_index++] = 0;
    }
  }

  return true;
}

bool Coprocessor::waitForDelimiter() {
  uint8_t byte{};
  while (m_serial_device->getInputBytes()) {
    byte = m_serial_device->readByte();
    if (byte == 0x00) {
      m_bytes_read = 0;
      m_state = EStates::READ_PAYLOAD;
      return true;
    }
  }

  return false;
}

bool Coprocessor::readPayload() {
  uint8_t byte{};

  std::string received_string{};
  while (m_serial_device->getInputBytes()) {
    byte = m_serial_device->readByte();
    received_string += std::to_string(byte) + " ";
    if (byte == 0x00) {
      m_state = EStates::VALIDATE_PACKAGE;
      std::cout << "Received package: " << received_string << std::endl;
      return true;
    }

    m_input_package[m_bytes_read++] = byte;
  }

  return false;
}

bool Coprocessor::validatePackage() {
  if (!decodeCOBS()) {
    m_state = EStates::ERROR;
    return true;
  }

  m_bytes_read = 0;
  m_expected_package_size = m_decoded_input_package[0];

  std::string decoded_string{};
  for (int i = 0; i < m_expected_package_size; ++i) {
    decoded_string += std::to_string(m_decoded_input_package[i]) + " ";
  }
  std::cout << "Decoded package: " << decoded_string << std::endl;

  uint16_t recieved_crc{};

  memcpy(&recieved_crc, &m_decoded_input_package[m_expected_package_size - 2],
         2);
  uint16_t calculated_crc{calculateCRC(m_decoded_input_package.data(),
                                       m_expected_package_size - 2)};

  if (recieved_crc == calculated_crc) {
    m_state = EStates::PROCESS_COMMANDS;
  } else {
    m_state = EStates::ERROR;
  }

  return true;
}

bool Coprocessor::processCommands() {
  uint8_t num_packets = m_decoded_input_package[1];
  uint8_t packet_index = 2;

  m_output_size = 2;
  m_output_packets = 0;

  for (uint8_t i = 0; i < num_packets; ++i) {
    uint8_t key = m_decoded_input_package[packet_index++];

    if (key >= m_packet_handlers.size() || m_packet_handlers[key] == nullptr) {
      m_state = EStates::ERROR;
      return true;
    }

    m_packet_handlers[key](&m_decoded_input_package[packet_index], this);
    packet_index += m_packet_sizes[key];
  }

  m_state = EStates::READ_PAYLOAD;
  return true;
}

bool Coprocessor::buildResponse() {
  m_outgoing_package.clearPackets();

  if (m_connection_established) {
    for (const auto& packet : m_recurring_packets) {
      m_outgoing_package.addPacket(packet);
    }

    for (const auto& packet : m_packet_queue) {
      m_outgoing_package.addPacket(packet);
    }
    m_packet_queue.clear();

  } else {
    serial_protocol::Packet connection_packet{serial_protocol::ESerialKey::ACK,
                                              {1}};
    m_outgoing_package.addPacket(connection_packet);
  }

  std::vector<uint8_t> serialized_package{
      m_outgoing_package.getSerializedPackage()};
  std::memcpy(m_output_buffer.data(), serialized_package.data(),
              serialized_package.size());
  m_output_size = serialized_package[0];

  uint16_t crc{calculateCRC(m_output_buffer.data(), m_output_size - 2)};
  memcpy(&m_output_buffer[m_output_size - 2], &crc, 2);

  encodeCOBS();
  m_output_buffer[m_output_size++] = 0x00;

  return true;
}

bool Coprocessor::sendResponse() {
  std::string output_string;
  for (size_t i = 0; i < m_output_size; ++i) {
    output_string += std::to_string(m_output_buffer[i]) + " ";
  }

  m_serial_device->write(m_output_buffer.data(), m_output_size);
  return true;
}

bool Coprocessor::handleError() {
  m_state = EStates::WAIT_FOR_DELIMITER;
  return false;
}

void Coprocessor::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  uint32_t start_time{m_clock->getTime()};

  bool update_state = true;
  while (update_state) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 10, "State: %d",
                        static_cast<int>(m_state));
    switch (m_state) {
      case EStates::WAIT_FOR_DELIMITER:
        update_state = waitForDelimiter();
        break;

      case EStates::READ_PAYLOAD:
        update_state = readPayload();
        break;

      case EStates::VALIDATE_PACKAGE:
        update_state = validatePackage();
        break;

      case EStates::PROCESS_COMMANDS:
        update_state = processCommands();
        break;

      case EStates::ERROR:
        update_state = handleError();
        break;
    }
  }

  buildResponse();
  sendResponse();

  if (m_mutex) {
    m_mutex->give();
  }

  uint32_t elapsed_time{m_clock->getTime() - start_time};
  if (elapsed_time > 0) {
    m_delayer->delay(Coprocessor::TASK_DELAY - elapsed_time);
  }
}

void Coprocessor::handleXposCommand(const uint8_t* data,
                                    Coprocessor* coprocessor) {
  if (data) {
    std::memcpy(coprocessor
                    ->m_latest_data[static_cast<uint8_t>(
                        serial_protocol::ESerialKey::XPOS)]
                    .data(),
                data, sizeof(float));
    coprocessor
        ->m_has_data[static_cast<uint8_t>(serial_protocol::ESerialKey::XPOS)] =
        true;
  }
}

void Coprocessor::handleYposCommand(const uint8_t* data,
                                    Coprocessor* coprocessor) {
  if (data) {
    std::memcpy(coprocessor
                    ->m_latest_data[static_cast<uint8_t>(
                        serial_protocol::ESerialKey::YPOS)]
                    .data(),
                data, sizeof(float));
    coprocessor
        ->m_has_data[static_cast<uint8_t>(serial_protocol::ESerialKey::YPOS)] =
        true;
  }
}

void Coprocessor::handleThetaCommand(const uint8_t* data,
                                     Coprocessor* coprocessor) {
  if (data) {
    std::memcpy(coprocessor
                    ->m_latest_data[static_cast<uint8_t>(
                        serial_protocol::ESerialKey::THETA)]
                    .data(),
                data, sizeof(float));
    coprocessor
        ->m_has_data[static_cast<uint8_t>(serial_protocol::ESerialKey::THETA)] =
        true;
  }
}

void Coprocessor::handlePosCommand(const uint8_t* data,
                                   Coprocessor* coprocessor) {
  if (data) {
    std::memcpy(coprocessor
                    ->m_latest_data[static_cast<uint8_t>(
                        serial_protocol::ESerialKey::POSITION)]
                    .data(),
                data, 6);
    coprocessor->m_has_data[static_cast<uint8_t>(
        serial_protocol::ESerialKey::POSITION)] = true;
  }
}

void Coprocessor::handleAckCommand(const uint8_t* data,
                                   Coprocessor* coprocessor) {
  coprocessor->m_connection_established = true;
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 6, "Connection established");
}

Coprocessor::Coprocessor() {
  m_packet_handlers[static_cast<uint8_t>(serial_protocol::ESerialKey::XPOS)] =
      &Coprocessor::handleXposCommand;
  m_packet_handlers[static_cast<uint8_t>(serial_protocol::ESerialKey::YPOS)] =
      &Coprocessor::handleYposCommand;
  m_packet_handlers[static_cast<uint8_t>(serial_protocol::ESerialKey::THETA)] =
      &Coprocessor::handleThetaCommand;
  m_packet_handlers[static_cast<uint8_t>(serial_protocol::ESerialKey::ACK)] =
      &Coprocessor::handleAckCommand;
  m_packet_handlers[static_cast<uint8_t>(
      serial_protocol::ESerialKey::POSITION)] = &Coprocessor::handlePosCommand;
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