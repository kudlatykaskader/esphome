#pragma once
#include <cinttypes>
// #include <Arduino.h>
// #include "esphome/core/log.h"
// #include "request.h"

namespace esphome {
namespace diesel_heater {

class SystemState {
 public:
  bool on;
  bool fan;
  bool spark_plug;
  bool pump;
  bool alpine;
  bool mode;
  bool cooling;
  uint16_t heat_exchanger_temp;
  uint8_t voltage;
  uint8_t heating_power;

  void toggle_power() { on = !on; }
  void toggle_alpine() { alpine = !alpine; }
  void toggle_mode() { mode = !mode; }

  void adjust_heating_power_up() {
    if (heating_power < 5) heating_power++;
  }

  void adjust_heating_power_down() {
    if (heating_power > 0) heating_power--;
  }
};

}  // namespace diesel_heater
}  // namespace esphome
