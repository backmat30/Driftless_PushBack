#include "PackageManager.h"

uint16_t PackageManager::calculateCRC(uint8_t* data, size_t size) {
  uint16_t crc{ 0xFFFF };
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



void PackageManager::encodeCOBS() {
  size_t read_index{};
  size_t write_index{ 1 };
  size_t latest_zero_index{};
  uint8_t bytes_since_zero{ 1 };

  while (read_index < m_output_size) {
    if (m_output_buffer[read_index] == 0) {
      m_output_package[latest_zero_index] = bytes_since_zero;
      bytes_since_zero = 1;
      latest_zero_index = write_index++;
      read_index++;

    } else {
      m_output_package[write_index++] = m_output_buffer[read_index++];
      bytes_since_zero++;

      if (bytes_since_zero == 0xFF) {
        m_output_package[latest_zero_index] = bytes_since_zero;
        bytes_since_zero = 1;
        latest_zero_index = write_index++;
      }
    }
  }

  m_output_size = write_index;
  m_output_package[latest_zero_index] = bytes_since_zero;
}



bool PackageManager::decodeCOBS() {
  size_t read_index{0};
  size_t write_index{};
  uint8_t bytes_until_zero{m_input_package[0]};

  while (read_index < m_input_size) {
    if (m_input_package[read_index] == 0 || m_expected_package_size + 1 > read_index + bytes_until_zero) {
      return false;
    }

    read_index++;

    for (uint8_t i = 1; i < bytes_until_zero; ++i) {
      m_decoded_input_package[write_index++] = m_input_package[read_index++];
    }

    if (bytes_until_zero != 0xFF && read_index < m_expected_package_size) {
      m_decoded_input_package[write_index++] = 0;
    }
  }

  m_output_size = write_index;
  return true;
}



bool PackageManager::WaitForDelimeter() {
  uint8_t byte{};
  while (m_input_buffer.readNext(byte)) {
    if (byte == 0x00) {
      m_bytes_read = 0;
      m_state = States::READ_SIZE;
      return true;
    }
  }

  return false;
}



bool PackageManager::readSize() {
  if (m_input_buffer.isEmpty()) {
    return false;
  }

  m_input_buffer.readNext(m_expected_package_size);

  if (m_expected_package_size < 4) {
    m_state = States::ERROR;
    return true;
  }

  m_input_package[m_bytes_read++] = m_expected_package_size;
  m_state = States::READ_PAYLOAD;
  return true;
}



bool PackageManager::readPayload() {
  uint8_t byte{};

  while (m_input_buffer.readNext(byte)) {
    if (byte == 0x00) {
      m_state = States::ERROR;
      return true;
    }

    m_input_package[m_bytes_read++] = byte;

    if (m_bytes_read == m_expected_package_size) {
      m_state = States::VALIDATE_PACKAGE;
      return true;
    }
  }

  return false;
}



bool PackageManager::validatePackage() {
  if (!decodeCOBS()) {
    m_state = States::ERROR;
    return true;
  }

  uint16_t recieved_crc{};

  memcpy(&recieved_crc, &m_decoded_input_package[m_expected_package_size - 2], 2);
  uint16_t calculated_crc{ calculateCRC(m_decoded_input_package.data(), m_expected_package_size - 2) };

  if (recieved_crc == calculated_crc) {
    m_state = States::PROCESS_COMMANDS;
  } else {
    m_state = States::ERROR;
  }

  return true;
}



bool PackageManager::processCommands() {
  uint8_t num_packets = m_decoded_input_package[1];
  uint8_t packet_index = 2;

  m_output_size = 2;
  m_output_packets = 0;

  for (uint8_t i = 0; i < num_packets; ++i) {
    uint8_t key = m_decoded_input_package[packet_index++];

    if (key >= m_packet_handlers.size() || m_packet_handlers[key] == nullptr) {
      m_state = States::ERROR;
      return true;
    }

    m_packet_handlers[key](&m_decoded_input_package[packet_index], m_output_buffer.data(), m_output_size, m_output_packets);
    packet_index += m_packet_sizes[key];
  }

  if (m_output_packets) {
    m_state = States::BUILD_RESPONSE;
  } else {
    m_state = States::WAIT_FOR_DELIMITER;
  }
  return true;
}



bool PackageManager::buildResponse() {
  m_output_size;
  m_output_buffer[0] = m_output_size + 2;
  m_output_buffer[1] = m_output_packets;

  uint16_t crc{ calculateCRC(m_output_buffer.data(), m_output_size) };
  memcpy(&m_output_buffer[m_output_size], &crc, 2);
  m_output_size += 2;

  encodeCOBS();

  m_state = States::SEND_RESPONSE;
  return true;
}



bool PackageManager::sendResponse() {
  if(!m_serial_port->availableForWrite()) {
    m_state = States::ERROR;
    return true;
  }

  for(size_t i = 0; i < m_output_size; ++i) {
    m_serial_port->write(m_output_package[i]);
  }

  m_state = States::WAIT_FOR_DELIMITER;
  return true;
}

bool PackageManager::handleError() {
  //TODO actually handle errors

  m_state = States::WAIT_FOR_DELIMITER;
  return true;
}



PackageManager::PackageManager(HardwareSerialIMXRT* serial_port)
  : m_serial_port{ serial_port } {}



void PackageManager::addPacketType(char key, uint8_t size,
                                   void (*)(const uint8_t* data) handler) {
  m_packet_sizes[static_cast<int>(key)] = size;
  m_packet_handlers[static_cast<int>(key)] = handler;
}



void PackageManager::update() {
  while (m_serial_port->available() && !m_input_buffer.isFull()) {
    uint8_t next_byte{ m_serial_port->read() };
    m_input_buffer.write(next_byte);
  }

  bool update_state = true;
  while (update_state) {
    switch (m_state) {
      case States::WAIT_FOR_DELIMITER:
        update_state = waitForDelimiter();
        break;

      case States::READ_SIZE:
        update_state = readSize();
        break;

      case States::READ_PAYLOAD:
        update_state = readPayload();
        break;

      case States::VALIDATE_PACKAGE:
        update_state = validatePackage();
        break;

      case States::PROCESS_COMMANDS:
        update_state = processCommands();
        break;

      case States::BUILD_RESPONSE:
        update_state = buildResponse();
        break;

      case States::SEND_RESPONSE:
        update_state = sendResponse();
        break;

      case States::ERROR:
        update_state = handleError();
        break;
    }
  }
}