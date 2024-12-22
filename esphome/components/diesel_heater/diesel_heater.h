#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"

#include "diesel_heater_state_machine.h"
#include "diesel_heater_platform.h"
#include "message_handlers.h"

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

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; };
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; };
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; };
  void set_mode_sensor(sensor::Sensor *mode_sensor) { mode_sensor_ = mode_sensor; };
  void set_alpine_sensor(sensor::Sensor *alpine_sensor) { alpine_sensor_ = alpine_sensor; };
  void set_fan_sensor(sensor::Sensor *fan_sensor) { fan_sensor_ = fan_sensor; };
  void set_pump_sensor(sensor::Sensor *pump_sensor) { pump_sensor_ = pump_sensor; };
  void set_spark_plug_sensor(sensor::Sensor *spark_plug_sensor) { spark_plug_sensor_ = spark_plug_sensor; };
  void set_power_switch(switch_::Switch *sw) { power_switch_ = sw; }
  void set_alpine_switch(switch_::Switch *sw) { alpine_switch_ = sw; }
  void set_mode_switch(switch_::Switch *sw) { mode_switch_ = sw; }
  void set_power_up_button(button::Button *button) { power_up_button_ = button; }
  void set_power_down_button(button::Button *button) { power_down_button_ = button; }

  // bool get_power_switch() { return this->system_state.on; }
  // void set_power_switch(bool state) { this->system_state.on = state; }
  bool set_power_switch_state(bool state) { 
    if (this->system_state.on != state) {
      this->system_state.on = state;
      this->request_queue_.push(RequestType::POWER_TOGGLE);
      return true;
    }
    return false;
  }

  bool set_mode_switch_state(bool state) { 
    if (this->system_state.mode != state) {
      this->system_state.mode = state;
      // this->request_queue_.push(RequestType::MODE_TOGGLE);
      return true;
    }
    return false;
  }

  bool set_alpine_switch_state(bool state) { 
    if (this->system_state.alpine != state) {
      this->system_state.alpine = state;
      this->request_queue_.push(RequestType::ALPINE_TOGGLE);
      return true;
    }
    return false;
  }

  void increase_power() { 
    this->system_state.adjust_heating_power_up();
    this->request_queue_.push(RequestType::POWER_UP);
  }
  
  void decrease_power() {
    this->system_state.adjust_heating_power_down();
    this->request_queue_.push(RequestType::POWER_DOWN);
  }

  static DieselHeater *instance_;

  // ISR handlers called by platform interface
  void IRAM_ATTR on_pin_isr();
  void IRAM_ATTR on_timer_isr();

 protected:
  InternalGPIOPin *data_pin_{nullptr};
  InternalGPIOPin *debug_pin_1_{nullptr};
  InternalGPIOPin *debug_pin_2_{nullptr};

  OperatingMode op_mode_{OperatingMode::MODE_SHARED};

  // State machine instance
  StateMachine sm_;

  PlatformInterface platform_{this};

  // Timing constants - adjust multipliers as needed
  static constexpr uint32_t TIME_PERIOD_4040us = 4040 * 5;
  static constexpr uint32_t TIME_PERIOD_30ms   = 30000 * 5;
  static constexpr uint32_t TIME_PERIOD_1700us = 1700 * 5;

  SystemState system_state = SystemState(0, 0, 0, 0, 0, 0, 0, 68, 14, 3);
  bool current_request[24] = {};
  bool current_response[48] = {};

  void start_data_read(uint8_t bits);
  void toggle_debug_pin(InternalGPIOPin *pin, uint32_t delay);

  void handle_reading_state();
  void handle_response_state();

  void decode_raw_request(); // future decoding logic

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *mode_sensor_{nullptr};
  sensor::Sensor *alpine_sensor_{nullptr};
  sensor::Sensor *fan_sensor_{nullptr};
  sensor::Sensor *pump_sensor_{nullptr};
  sensor::Sensor *spark_plug_sensor_{nullptr};

  switch_::Switch *power_switch_{nullptr};
  switch_::Switch *alpine_switch_{nullptr};
  switch_::Switch *mode_switch_{nullptr};

  button::Button *power_up_button_{nullptr};
  button::Button *power_down_button_{nullptr};

  // update internal in milliseconds
  uint32_t update_interval_{5 * 1000};
  uint32_t last_update_{0};

  // create a command queue for handling requests
  std::queue<RequestType> request_queue_;
};

}  // namespace diesel_heater
}  // namespace esphome
