#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"
#include <map>
#include <vector>

/**
 * Arduino code to recieve odometry information from a
 * Sparkfun Odometry sensor (PAA5160E1) and transmits the
 * information to the Vex brain over RS485 connection
 *
 * @author Matthew Backman
 */

constexpr char START_DELIMETER{'/'};
std::map<char, uint8_t> key_size{{'R', 1}, 
                                  {'X', 4}, 
                                  {'Y', 4}, 
                                  {'H', 4}};
// Create an Optical Tracking Odometry Sensor object
QwiicOTOS odom_sensor;
Serial rs485Serial{ 20, 21 };

void setup() {
  // put your setup code here, to run once:

  rs485Serial.begin(921600);
  //Serial.begin(74880);
  Wire.begin();

  odom_sensor.setAngularUnit(kSfeOtosAngularUnitDegrees);
  odom_sensor.setLinearUnit(kSfeOtosLinearUnitInches);

  odom_sensor.resetTracking();
}

void loop() {
  // put your main code here, to run repeatedly:

  uint64_t current_time{ millis() };
  bool should_send{};
  std::vector<char> packets_requested{};

  // Check for input from brain
  if (rs485Serial.available()) {
    char* recieved_data;
    int num_bytes{rs485Serial.available()};
    rs485Serial.readBytes(recieved_data, num_bytes);
    uint8_t packets_recieved{recieved_data[0]};

    uint8_t packet_start_index{1};
    for(int i = 0; i < packets_recieved; ++i) {
      char key{recieved_data[packet_start_index++]};
      if(key_size.find(key) != key_size.cend()) {
        break;
      }
      uint8_t size{key_size.at(key)};

      // validate data
      if(isValidPackage(recieved_data, num_bytes)) {
        break;
      }

      // handle data
      switch(key) {
        case 'R': {
          should_send = true;

          char requested_key{};
          memcpy(&requested_key, &recieved_data[packet_start_index], size);
          packets_requested.push_back(requested_key);
          break;
        }
        case 'C': {
          odom_sensor.calibrateImu();
          delay(1000);
          break;
        }
        case 'X': {
          float x_offset{};
          memcpy(&x_offset, &recieved_data[packet_start_index], size);

          sfe_otos_pose2d_t offset{};
          odom_sensor.getOffset(offset);
          offset.x = x_offset;
          odom_sensor.setOffset(offset);
          break;
        }
        case 'Y': {
          float y_offset{};
          memcpy(&y_offset, &recieved_data[packet_start_index], size);

          sfe_otos_pose2d_t offset{};
          odom_sensor.getOffset(offset);
          offset.y = y_offset;
          odom_sensor.setOffset(offset);
          break;
        }
        case 'H': {
          float h_offset{};
          memcpy(&h_offset, &recieved_data[packet_start_index], size);

          sfe_otos_pose2d_t offset{};
          odom_sensor.getOffset(offset);
          offset.h = h_offset;
          odom_sensor.setOffset(offset);
          break;
        }
        
      }

    }

  }

  // generate package of data if requested
  if(should_send) {
    String output_string{static_cast<char>(packets_requested.size())};
    for(char& requested_key : packets_requested) {
      output_string.append(requested_key);

      switch(requested_key) {
        case 'X': {
          sfe_otos_pose2d_t position;
          odom_sensor.getPosition(position);

          char value[key_size.at(requested_key)];
          memcpy(&value, &position.x, key_size.at(requested_key));

          output_string.append(value);
          break;
        }
        case 'Y': {
          sfe_otos_pose2d_t position;
          odom_sensor.getPosition(position);

          char value[key_size.at(requested_key)];
          memcpy(&value, &position.y, key_size.at(requested_key));

          output_string.append(value);
          break;
        }
        case 'H': {
          sfe_otos_pose2d_t position;
          odom_sensor.getPosition(position);

          char value[key_size.at(requested_key)];
          memcpy(&value, &position.h, key_size.at(requested_key));

          output_string.append(value);
          break;
        }
      }
    }
    // calculate crc for validation
    uint16_t crc{computeCRC(output_string.c_str(), output_string.length())};
    char crc_bytes[2];
    memcpy(&crc_bytes, &crc, 2);
    output_string.append(crc_bytes);

    rs485Serial.print(output_string);
  }

  
  sfe_otos_pose2d_t position;  

  /* SEND DATA TO ARDUINO IDE FOR DEBUG, UNCOMMENT IF NEEDED */
  //Serial.println(output_string);

  uint64_t delay_time{ 10 - (millis() - current_time) };
  if (delay_time > 0) {
    delay(delay_time);
  }
}

/** Builds a packet to send over the serial link
 * @tparam T    The datatype of the value sent
 * @param key   The key designating the purpose of the data
 * @param value The value held by the packet 
 */
template<typename T>
String buildPacket(char key, T value) {
  char size{sizeof(T)};
  char bytes[size]{};
  memcpy(&bytes, &value, size);

  String delimeter_str{START_DELIMETER};
  String key_str{key};
  String size_str{size};
  String value_str{bytes};

  String output{delimeter_str + key_str + size_str + value_str};
  return output;
}

/** Computes the CRC of a package
* @param data the base package
* @param length the size of the package
* @return CRC value
*/
uint16_t computeCRC(const char* data, size_t length) {
  uint16_t crc{0xFFFF};
  for (size_t i = 0; i < length; ++i) {
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

/** Validates a package by checking the CRC value
* @param data The package recieved
* @param length The size of the package
* @return True if the packet was transmitted correctly
*/
bool isValidPackage(const char* data, size_t length) {
  uint16_t actual_crc{};

  memcpy(&actual_crc, &data[length-2], 2);
  uint16_t calculated_crc{computeCRC(data, length - 2)};

  return actual_crc == calculated_crc;
}
