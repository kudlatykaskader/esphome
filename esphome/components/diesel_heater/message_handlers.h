// #pragma once
// #include <cinttypes>
// #include <Arduino.h>
// #include "esphome/core/log.h"
// #include "request.h"

// namespace esphome {
// namespace diesel_heater {

// class SystemState {
//  public:
//   bool on;
//   bool fan;
//   bool spark_plug;
//   bool pump;
//   bool alpine;
//   bool mode;            // represent different mode states, just a boolean toggle for example
//   bool cooling;
//   uint16_t heat_exchanger_temp;  // 0-1023 fits in 10 bits
//   uint8_t voltage;               // 0-63 fits in 6 bits
//   uint8_t heating_power;         // a value representing heating power level (0-63?)

//   void toggle_power() { on = !on; }
//   void toggle_alpine() { alpine = !alpine; }
//   void toggle_mode() { mode = !mode; }

//   void adjust_heating_power_up() {
//     if (heating_power < 5) {
//       heating_power++;
//       ESP_LOGD("", "Heating power: %d", heating_power);
//     }
//   }

//   void adjust_heating_power_down() {
//     if (heating_power > 0) {
//       heating_power--;
//       ESP_LOGD("", "Heating power: %d", heating_power);
//     }
//   }
// };

// inline Response generate_response(RequestType req_type, const SystemState &state, uint8_t request_data) {
//   if (req_type == RequestType::STATUS_IDLE 
//       || req_type == RequestType::POWER_TOGGLE 
//       || req_type == RequestType::POWER_DOWN 
//       || req_type == RequestType::POWER_UP
//       || req_type == RequestType::ALPINE_TOGGLE
//       || req_type == RequestType::PUMP_PRIME
//       || req_type == RequestType::MODE_TOGGLE) {
//     uint16_t response = 0b0110000000000000;

//     if (!state.on) {
//       return Response(response | (state.voltage & 0x3F));
//     } else {
//       // response |= state.heating_power & 0b001;
//       // response |= state.heating_power & 0b010 << 1;
//       // response |= state.heating_power & 0b100 << 2;
//       // response |= (state.mode ? 1 : 0)        << 5;
//       response |= (state.on ? 1 : 0)          << 6;
//       response |= (state.cooling ? 1 : 0)     << 7;
//       response |= (state.spark_plug ? 1 : 0)  << 8;
//       response |= (state.alpine ? 1 : 0)      << 9;
//       response |= (state.pump ? 1 : 0)        << 10;
//       response |= (state.fan ? 1 : 0)         << 11;
//       return Response(response);
//     }
//   } else if (req_type == RequestType::VOLTAGE) {
//     return Response(0b1010000000000000 | (state.voltage & 0x3F));
//   } else if (req_type == RequestType::TEMPERATURE) {
//     // return Response(0b100000000000000 | (400 & 0x3F);
//     return Response(0b0100001111011111);
//   } else {
//     // Unknown request, return 0
//     return Response(0);
//   }
// }

// // Given the 24-bit request read from wire, decode and handle it:
// inline void handle_request(const bool *request24bits, bool *response48bits, SystemState &state) {
//   uint8_t request_data = decode_byte(request24bits);
//   RequestType req_type = identify_request(request_data);

//   // React to incoming messages by changing system state if needed
//   switch (req_type) {
//     case RequestType::POWER_TOGGLE:
//       state.toggle_on();
//       break;
//     case RequestType::POWER_DOWN:
//       state.adjust_heating_power_down();
//       break;
//     case RequestType::POWER_UP:
//       state.adjust_heating_power_up();
//       break;
//     case RequestType::ALPINE_TOGGLE:
//       state.toggle_alpine();
//       break;
//     case RequestType::PUMP_PRIME:
//       state.pump = true;
//       break;
//     case RequestType::MODE_TOGGLE:
//       state.toggle_mode();
//       break;
//     case RequestType::UNKNOWN:
//     default:
//       // No state change
//       break;
//   }
//   // Now generate a response based on the new state and request
//   Response resp = generate_response(req_type, state, request_data);

//   // Encode the 16-bit response into 48 bits
//   encode_16bits(resp.data, response48bits);

//   // ESP_LOGD("", "Response bits: %i", resp.data & 0b0000000000001111);
  

//   ESP_LOGD("", "Syste State: O:%d F:%d S:%d P:%d A:%d M:%d C:%d H:%d V:%d PR:%d", state.on, state.fan, state.spark_plug, state.pump, state.alpine, state.mode, state.cooling, state.heat_exchanger_temp, state.voltage, state.heating_power);
//   ESP_LOGD("", "Request: 0x%02X, Response: 0x%04X", request_data, resp.data);

// }

// inline void handle_response(const bool *response48bits, SystemState &state) {
//   uint16_t response_data = 0;
//   for (int i = 0; i < 16; i++) {
//     response_data |= response48bits[i * 3 + 1] << (15 - i);
//   }

//   // Decode response and update system state
//   // First 4 bits represent a response type, the rest is data
//   // When response type is 0b0110, the rest of the data is system state
//   // When response type is 0b1010, the rest of the data is voltage
//   // When response type is 0b1000, the rest of the data is temperature
//   uint8_t response_type = (response_data & 0xF000) >> 12;
//   uint16_t response_value = response_data & 0x0FFF;
//   ESP_LOGD("", "SystemState[ON]: %d", state.on);
//   switch (response_type) {
//     case 0b0110:
//       state.on = (response_value >> 6 & 0b1);
//       state.cooling = (response_value >> 7 & 0b1);
//       state.mode = (response_value >> 5 & 0b1);
//       state.spark_plug = (response_value >> 8 & 0b1);
//       state.alpine = (response_value >> 9 & 0b1);
//       state.pump = (response_value >> 10 & 0b1);
//       state.fan = (response_value >> 11 & 0b1);
//       state.heating_power = response_value & 0x7;
//       break;
//     case 0b1010:
//       state.voltage = response_value & 0x3F;
//       break;
//     case 0b1000:
//       // state.heat_exchanger_temp = response_value & 0x3FF;
//       break;
//     default:
//       // Unknown response type
//       break;
//     ESP_LOGD("", "SystemState[ON]: %d", state.on);
//   }
// }

// inline void generate_request(RequestType req_type, bool *request24bits) {
//   uint8_t request_data = 0;
//   switch (req_type) {
//     case RequestType::POWER_TOGGLE:
//       request_data = 0xA2;
//       break;
//     case RequestType::POWER_DOWN:
//       request_data = 0xA4;
//       break;
//     case RequestType::POWER_UP:
//       request_data = 0xA3;
//       break;
//     case RequestType::ALPINE_TOGGLE:
//       request_data = 0xAE;
//       break;
//     case RequestType::PUMP_PRIME:
//       request_data = 0xAC;
//       break;
//     case RequestType::MODE_TOGGLE:
//       request_data = 0xA1;
//       break;
//     case RequestType::UNKNOWN:
//     default:
//       // No state change
//       break;
//   }
//   // Encode the 8-bit request into 24 bits
//   encode_byte(request_data, request24bits);
// }

// }  // namespace diesel_heater
// }  // namespace esphome
