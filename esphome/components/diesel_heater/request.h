#pragma once
#include <cinttypes>

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
 return ResponseType::UNKNOWN;
}

inline RequestType identify_request(uint8_t req) {
  switch (req) {
    // case 0xAF: return RequestType::STATUS_TEMP;
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

struct Response {
  uint16_t data;
  uint8_t bit_length;  // always 16 for now
  Response(uint16_t d, uint8_t len = 16) : data(d), bit_length(len) {}
};

}  // namespace diesel_heater
}  // namespace esphome