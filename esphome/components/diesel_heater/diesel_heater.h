#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/core/automation.h"

#include "state_machine.h"
#include "message_handlers.h"
#include "dh_protocol.h"
#include "state.h"
#include "request.h"

namespace esphome {
namespace diesel_heater {

class DieselHeater : public Component {
 public:
  // built in methods
  void setup() override;
  void loop() override;
  void dump_config() override;

  // automations
  void enable();
  void disable();

  // configuration: pins
  InternalGPIOPin *data_pin_{nullptr};
  
  void set_data_pin(InternalGPIOPin *data_pin) { data_pin_ = data_pin; }

  // configuration: sensors
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *mode_sensor_{nullptr};
  sensor::Sensor *alpine_sensor_{nullptr};
  sensor::Sensor *fan_sensor_{nullptr};
  sensor::Sensor *pump_sensor_{nullptr};
  sensor::Sensor *spark_plug_sensor_{nullptr};
  sensor::Sensor *cooling_sensor_{nullptr};

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; };
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; };
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; };
  void set_mode_sensor(sensor::Sensor *mode_sensor) { mode_sensor_ = mode_sensor; };
  void set_alpine_sensor(sensor::Sensor *alpine_sensor) { alpine_sensor_ = alpine_sensor; };
  void set_fan_sensor(sensor::Sensor *fan_sensor) { fan_sensor_ = fan_sensor; };
  void set_pump_sensor(sensor::Sensor *pump_sensor) { pump_sensor_ = pump_sensor; };
  void set_spark_plug_sensor(sensor::Sensor *spark_plug_sensor) { spark_plug_sensor_ = spark_plug_sensor; };
  void set_cooling_sensor(sensor::Sensor *cooling_sensor) { cooling_sensor_ = cooling_sensor; };

  // configuration: switches  
  switch_::Switch *power_switch_{nullptr};
  switch_::Switch *alpine_switch_{nullptr};
  switch_::Switch *mode_switch_{nullptr};

  void set_power_switch(switch_::Switch *sw) { power_switch_ = sw; }
  bool set_power_switch_state(bool state);
  void set_alpine_switch(switch_::Switch *sw) { alpine_switch_ = sw; }
  bool set_alpine_switch_state(bool state); 
  void set_mode_switch(switch_::Switch *sw) { mode_switch_ = sw; }
  bool set_mode_switch_state(bool state);


  // configuration: buttons
  button::Button *power_up_button_{nullptr};
  button::Button *power_down_button_{nullptr};

  void set_power_up_button(button::Button *button) { power_up_button_ = button; }
  void set_power_up_button_clicked();
  void set_power_down_button(button::Button *button) { power_down_button_ = button; }
  void set_power_down_button_clicked();

  // interrupts
  static DieselHeater *instance_;
  static IRAM_ATTR void on_request_callback(uint8_t data);
  static IRAM_ATTR void on_response_callback(uint16_t data);
  
 protected:
  DHProtocol *protocol_;

  SystemState system_state = SystemState();
  uint8_t request = 0;
  uint16_t response = 0;
  bool data_pending = false;

  uint32_t temp_req_sent_timestamp = 0;
  uint32_t temp_req_sent_period = 3 * 1000;
  
  uint32_t voltage_req_sent_timestamp = 0;
  uint32_t voltage_req_sent_period = 10 * 1000;

  void on_request_received(uint8_t data);
  void on_response_received(uint16_t data);
};

template<typename... Ts> class DieselHeaterDisableAction : public Action<Ts...> {
 public:
  void play(Ts... x) override { DieselHeater::instance_->disable(); }
};

}  // namespace diesel_heater
}  // namespace esphome
