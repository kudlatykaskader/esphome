#pragma once
#include <cinttypes>
#include "state.h"
#include "temperatures.h"
#include "esphome/core/log.h"

#define U8_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define U8_TO_BINARY(byte)  \
  ((byte) & 0b10000000 ? '1' : '0'), \
  ((byte) & 0b01000000 ? '1' : '0'), \
  ((byte) & 0b00100000 ? '1' : '0'), \
  ((byte) & 0b00010000 ? '1' : '0'), \
  ((byte) & 0b00001000 ? '1' : '0'), \
  ((byte) & 0b00000100 ? '1' : '0'), \
  ((byte) & 0b00000010 ? '1' : '0'), \
  ((byte) & 0b00000001 ? '1' : '0') 

#define U16_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
#define U16_TO_BINARY(byte)  \
  ((byte) & 0b1000000000000000 ? '1' : '0'), \
  ((byte) & 0b0100000000000000 ? '1' : '0'), \
  ((byte) & 0b0010000000000000 ? '1' : '0'), \
  ((byte) & 0b0001000000000000 ? '1' : '0'), \
  ((byte) & 0b0000100000000000 ? '1' : '0'), \
  ((byte) & 0b0000010000000000 ? '1' : '0'), \
  ((byte) & 0b0000001000000000 ? '1' : '0'), \
  ((byte) & 0b0000000100000000 ? '1' : '0'), \
  ((byte) & 0b0000000010000000 ? '1' : '0'), \
  ((byte) & 0b0000000001000000 ? '1' : '0'), \
  ((byte) & 0b0000000000100000 ? '1' : '0'), \
  ((byte) & 0b0000000000010000 ? '1' : '0'), \
  ((byte) & 0b0000000000001000 ? '1' : '0'), \
  ((byte) & 0b0000000000000100 ? '1' : '0'), \
  ((byte) & 0b0000000000000010 ? '1' : '0'), \
  ((byte) & 0b0000000000000001 ? '1' : '0')

namespace esphome {
namespace diesel_heater {

enum class RequestType {
  UNKNOWN = 0,
  VOLTAGE = 0xAB,
  TEMPERATURE = 0xAF,    // 0xAF
  PUMP_PRIME = 0xAC,     // 0xAC
  POWER_DOWN = 0xA4,     // 0xA4
  POWER_UP = 0xA3,       // 0xA3
  ALPINE_TOGGLE = 0xAE,  // 0xAE
  POWER_TOGGLE = 0xA2,   // 0xA2
  STATUS_IDLE = 0xA6,    // 0xA6
  MODE_TOGGLE = 0xA1     // 0xA1
};

enum class ResponseType {
  UNKNOWN = 0,
  TEMP = 0x04,
  STATUS = 0x06,
  VOLTAGE = 0xA0,
};

inline ResponseType identify_response(uint16_t resp) {
  switch ((uint8_t)(resp >> 12)) {
    case 0x04:
      return ResponseType::TEMP;
    case 0x06:
      return ResponseType::STATUS;
    case 0xA0:
      return ResponseType::VOLTAGE;
    default:
      return ResponseType::UNKNOWN;
  }
}

inline RequestType identify_request(uint8_t req) {
  switch (req) {
    case 0xAB:
      return RequestType::VOLTAGE;
    case 0xAF:
      return RequestType::TEMPERATURE;
    case 0xAC:
      return RequestType::PUMP_PRIME;
    case 0xA4:
      return RequestType::POWER_DOWN;
    case 0xA3:
      return RequestType::POWER_UP;
    case 0xAE:
      return RequestType::ALPINE_TOGGLE;
    case 0xA2:
      return RequestType::POWER_TOGGLE;
    case 0xA6:
      return RequestType::STATUS_IDLE;
    case 0xA1:
      return RequestType::MODE_TOGGLE;
    default:
      return RequestType::UNKNOWN;
  }
}

inline void parse_response(uint16_t response_data, SystemState &state) {
  uint8_t response_type = (response_data & 0xF000) >> 12;
  uint16_t response_value = response_data & 0x0FFF;
  switch (response_type) {
    case 0b0110:
      if (state.on) {
        state.on = (response_value >> 6 & 0b1);
        state.cooling = (response_value >> 7 & 0b1);
        state.mode = (response_value >> 5 & 0b1);
        state.spark_plug = (response_value >> 8 & 0b1);
        state.alpine = (response_value >> 9 & 0b1);
        state.pump = (response_value >> 10 & 0b1);
        state.fan = (response_value >> 11 & 0b1);
        state.heating_power = response_value & 0x7;
      } else {
        state.voltage = response_value & 0x3F;  
      }
      break;
    case 0b1010:
      state.voltage = response_value & 0x3F;
      break;
    case 0b0100:
      state.heat_exchanger_temp = get_temperature(response_value & 0x3FF);
      break;
    default:
      // Unknown response type
      break;
  }
}

inline void parse_request(uint8_t request_data, SystemState &state) {
  switch (request_data) {
    case 0xAB:
      // Voltage request
      break;
    case 0xAF:
      // Temperature request
      break;
    case 0xAC:
      // Pump prime request
      break;
    case 0xA4:
      // Power down request
      break;
    case 0xA3:
      // Power up request
      break;
    case 0xAE:
      // Alpine toggle request
      break;
    case 0xA2:
      // Power toggle request
      break;
    case 0xA6:
      // Status idle request
      break;
    case 0xA1:
      // Mode toggle request
      break;
    default:
      // Unknown request type
      break;
  }
}

}  // namespace diesel_heater
}  // namespace esphome