#include <array>

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"
#include "PackageManager.h"

// #define DEBUG

/**
 * Arduino code to recieve odometry information from a
 * Sparkfun Odometry sensor (PAA5160E1) and transmits the
 * information to the Vex brain over RS485 connection
 *
 * @author Matthew Backman
 */

void handleRequestCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num);
void handleCalibrateCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num);
void handleSetXCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num);
void handleSetYCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num);
void handleSetHeadingCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num);
void handleAckCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num);

std::array<uint8_t, 128> key_sizes{};

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS odom_sensor;

PackageManager manager{&Serial8};

void setup() {
  manager.addPacketType('R', 1, handleRequestCommand);
  manager.addPacketType('X', 4, handleSetXCommand);
  manager.addPacketType('Y', 4, handleSetYCommand);
  manager.addPacketType('H', 4, handleSetHeadingCommand);
  manager.addPacketType('C', 1, handleCalibrateCommand);
  manager.addPacketType('A', 1, handleAckCommand);

  key_sizes['R'] = 1;
  key_sizes['X'] = 4;
  key_sizes['Y'] = 4;
  key_sizes['H'] = 4;
  key_sizes['C'] = 1;
  key_sizes['A'] = 1;

  Serial8.begin(115200);

#ifdef DEBUG
  Serial.begin(74880);
#endif

  Wire2.begin();
  while (!(odom_sensor.begin(Wire2))) {
#ifdef DEBUG
    Serial.println("Odom initializing");
#endif
    delay(1000);
  }

  odom_sensor.setAngularUnit(kSfeOtosAngularUnitDegrees);
  odom_sensor.setLinearUnit(kSfeOtosLinearUnitInches);

  odom_sensor.resetTracking();
}

void loop() {
  manager.update();

  uint32_t end_time{millis()};
  uint32_t next_start_time{end_time + 10 - (end_time % 10)};
  
  delay(next_start_time - millis());
}

// Define command handlers
void handleRequestCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num) {
  char requested_key{};
  memcpy(&requested_key, data, 1);

  write_data[write_index++] = requested_key;

  uint8_t value_size{key_sizes[requested_key]};
  uint8_t value[value_size];
  switch (requested_key) {
    case 'X': {
      sfe_otos_pose2d_t position;
      odom_sensor.getPosition(position);
      float xPos = position.x;

      memcpy(&value, &xPos, value_size);
      break;
    }
    case 'Y': {
      sfe_otos_pose2d_t position;
      odom_sensor.getPosition(position);
      float yPos = position.y;

      memcpy(&value, &yPos, value_size);
      break;
    }
    case 'H': {
      sfe_otos_pose2d_t position;
      odom_sensor.getPosition(position);
      float heading = position.h;

      memcpy(&value, &heading, value_size);
      break;
    }
  }

  memcpy(write_data + write_index, value, value_size);
  write_index += value_size;
}

void handleCalibrateCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num) {
  odom_sensor.resetTracking();
  odom_sensor.calibrateImu(255);
#ifdef DEBUG
  Serial.println("Calibrating");
#endif
}

void handleSetXCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num) {
  float x_offset{};
  memcpy(&x_offset, data, 4);

#ifdef DEBUG
  Serial.println("X offset: " + String(x_offset));
#endif

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.x = x_offset;
  odom_sensor.setOffset(offset);
}

void handleSetYCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num) {
  float y_offset{};
  memcpy(&y_offset, data, 4);

#ifdef DEBUG
  Serial.println("Y offset: " + String(y_offset));
#endif

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.y = y_offset;
  odom_sensor.setOffset(offset);
}

void handleSetHeadingCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num) {
  float h_offset{};
  memcpy(&h_offset, data, 4);

#ifdef DEBUG
  Serial.println("Heading offset: " + String(h_offset));
#endif

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.h = h_offset;
  odom_sensor.setOffset(offset);
}

void handleAckCommand(const uint8_t* data, uint8_t* write_data, size_t& write_index, uint8_t& outgoing_packet_num) {
  outgoing_packet_num++;
  write_data[write_index++] = 'A';
  write_data[write_index++] = 0x00;
}