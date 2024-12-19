#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "diesel_heater_state_machine.h"
#include "diesel_heater_platform.h"

namespace esphome {
namespace diesel_heater {

class DieselHeater : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_data_pin(InternalGPIOPin *data_pin) { data_pin_ = data_pin; }
  void set_debug_pin_1(InternalGPIOPin *debug_pin) { debug_pin_1_ = debug_pin; }
  void set_debug_pin_2(InternalGPIOPin *debug_pin) { debug_pin_2_ = debug_pin; }

  static DieselHeater *instance_;

  // ISR handlers called by platform interface
  void IRAM_ATTR on_pin_isr();
  void IRAM_ATTR on_timer_isr();

 protected:
  InternalGPIOPin *data_pin_{nullptr};
  InternalGPIOPin *debug_pin_1_{nullptr};
  InternalGPIOPin *debug_pin_2_{nullptr};

  OperatingMode op_mode_{OperatingMode::MODE_HEATER};

  // State machine instance
  StateMachine sm_;

  PlatformInterface platform_{this};

  // Timing constants - adjust multipliers as needed
  static constexpr uint32_t TIME_PERIOD_4040us = 4040 * 5;
  static constexpr uint32_t TIME_PERIOD_30ms   = 30000 * 5;
  static constexpr uint32_t TIME_PERIOD_1700us = 1700 * 5;

  bool idle_response_[48] = {
    1,0,0,1,1,0,1,1,
    0,1,0,0,1,0,0,1,
    0,0,1,0,0,1,0,0,
    1,0,0,1,0,0,1,0,
    0,1,0,0,1,1,0,1,
    1,0,1,0,0,1,0,0
  };

  void start_data_read();
  void toggle_debug_pin(InternalGPIOPin *pin, uint32_t delay);

  void handle_reading_state();
  void handle_response_state();

  void decode_raw_request(); // future decoding logic
};

}  // namespace diesel_heater
}  // namespace esphome
