#pragma once

#include <cinttypes>
#include <string>

namespace esphome {
namespace diesel_heater_ble {

enum class HeaterClass {
  HEATER_AA_55,            // The one that sends responses with 0xAA 0x55 header
  HEATER_AA_66,            // The one that sends responses with 0xAA 0x66 header
  HEATER_AA_55_ENCRYPTED,  // The one that sends responses with 0xAA 0x55 header and encrypted data
  HEATER_AA_66_ENCRYPTED,  // The one that sends responses with 0xAA 0x66 header and encrypted data
  HEATER_CLASS_UNKNOWN
};

class HeaterState {
 public:
  HeaterClass heater_class;
  uint8_t rcv_cmd;
  uint8_t runningstate;
  uint8_t errcode;
  uint8_t runningstep;
  uint16_t altitude;
  uint8_t runningmode;
  uint8_t setlevel;
  uint8_t settemp;
  uint16_t supplyvoltage;
  uint16_t casetemp;
  uint16_t cabtemp;

  // encoded types only
  uint16_t sttime;
  uint16_t autotime;
  uint16_t runtime;
  uint8_t isauto;
  uint8_t language;
  uint8_t tempoffset;
  uint8_t tankvolume;
  uint8_t oilpumptype;
  bool rf433onoff;
  uint8_t tempunit;
  uint8_t altiunit;
  uint8_t automaticheating;

  // return table format of data with columnt names
  std::string to_string() {
    return "HeaterState: \n"
           "  heater_class: " +
           std::to_string(static_cast<int>(heater_class)) +
           "\n"
           "  rcv_cmd: " +
           std::to_string(rcv_cmd) +
           "\n"
           "  runningstate: " +
           std::to_string(runningstate) +
           "\n"
           "  errcode: " +
           std::to_string(errcode) +
           "\n"
           "  runningstep: " +
           std::to_string(runningstep) +
           "\n"
           "  altitude: " +
           std::to_string(altitude) +
           "\n"
           "  runningmode: " +
           std::to_string(runningmode) +
           "\n"
           "  setlevel: " +
           std::to_string(setlevel) +
           "\n"
           "  settemp: " +
           std::to_string(settemp) +
           "\n"
           "  supplyvoltage: " +
           std::to_string(supplyvoltage) +
           "\n"
           "  casetemp: " +
           std::to_string(casetemp) +
           "\n"
           "  cabtemp: " +
           std::to_string(cabtemp) +
           "\n"
           "  sttime: " +
           std::to_string(sttime) +
           "\n"
           "  autotime: " +
           std::to_string(autotime) +
           "\n"
           "  runtime: " +
           std::to_string(runtime) +
           "\n"
           "  isauto: " +
           std::to_string(isauto) + "\n";
  }
};

}  // namespace diesel_heater_ble
}  // namespace esphome

// // Represents the current system state
// class HeaterState {
//  public:
//   // Decoded fields from the response
//   uint16_t deviceStatusFlags;    // Device status flags (on/off, error states)
//   uint16_t tempSetpoint;         // Temperature setpoint
//   uint16_t currentTemperature;   // Current temperature
//   uint16_t altitude;             // Altitude in meters or feet
//   uint8_t batteryVoltage;        // Battery voltage (scaled)
//   uint8_t fuelLevel;             // Remaining fuel level
//   uint8_t operatingMode;         // Current operating mode
//   uint8_t errorCode;             // Error code
//   uint32_t runtimeCounter;       // Total runtime (in minutes or seconds)
//   uint8_t additionalStatus[10];  // Placeholder for additional statuses

//   // Decode raw response into meaningful fields

//   static std::vector<uint8_t> decode(const std::vector<uint8_t> &raw) {
//     if (raw.size() != 48 || raw[0] != 0xAA || raw[1] != 0x55) {
//       throw std::invalid_argument("Invalid packet format");
//     }

//     std::vector<uint8_t> decoded;
//     decoded.push_back(raw[3] | (raw[4] << 8));
//     decoded.push_back(raw[5] | (raw[6] << 8));
//     decoded.push_back(raw[7] | (raw[8] << 8));
//     decoded.push_back(raw[9] | (raw[10] << 8));
//     decoded.push_back(raw[11]);
//     decoded.push_back(raw[12]);
//     decoded.push_back(raw[13]);
//     decoded.push_back(raw[14]);
//     decoded.push_back(raw[15] | (raw[16] << 8) | (raw[17] << 16) | (raw[18] << 24));
//     std::memcpy(&decoded[9], &raw[19], 10);
//     return decoded;
//   }
//   static HeaterState decompose(const std::vector<uint8_t> &raw) {
//     if (raw.size() != 48 || raw[0] != 0xAA || raw[1] != 0x55) {
//       throw std::invalid_argument("Invalid packet format");
//     }

//     HeaterState state;
//     state.deviceStatusFlags = raw[3] | (raw[4] << 8);
//     state.tempSetpoint = raw[5] | (raw[6] << 8);
//     state.currentTemperature = raw[7] | (raw[8] << 8);
//     state.altitude = raw[9] | (raw[10] << 8);
//     state.batteryVoltage = raw[11];
//     state.fuelLevel = raw[12];
//     state.operatingMode = raw[13];
//     state.errorCode = raw[14];
//     state.runtimeCounter = raw[15] | (raw[16] << 8) | (raw[17] << 16) | (raw[18] << 24);
//     std::memcpy(state.additionalStatus, &raw[19], 10);
//     return state;
//   }
// };
