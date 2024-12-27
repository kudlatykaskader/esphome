#include "diesel_heater.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include <Arduino.h>

namespace esphome {
namespace diesel_heater {

static const char *const TAG = "diesel_heater";
DieselHeater *DieselHeater::instance_ = nullptr;

void DieselHeater::dump_config() {
  ESP_LOGCONFIG(TAG, "DieselHeater:");
  LOG_PIN("  Data Pin: ", this->data_pin_);
}

void DieselHeater::setup() {
  DieselHeater::instance_ = this;
  
  this->protocol_ = new DHProtocol(new InterruptPlatform(), this->data_pin_);
  this->protocol_->attach_request_received_callback(DieselHeater::on_request_callback);
  this->protocol_->attach_response_received_callback(DieselHeater::on_response_callback);
  this->protocol_->start();
}

void DieselHeater::loop() {
  if (this->data_pending) {
    this->data_pending = false;
    ESP_LOGD(TAG, "Data: " U8_TO_BINARY_PATTERN "->" U16_TO_BINARY_PATTERN, U8_TO_BINARY(this->request), U16_TO_BINARY(this->response));
  }

  if (this->system_state.on) {
    if (millis() - this->temp_req_sent_timestamp > this->temp_req_sent_period) {
      this->temp_req_sent_timestamp = millis();
      this->protocol_->add_request(0xAF);
    }
    if (millis() - this->voltage_req_sent_timestamp > this->voltage_req_sent_period) {
      this->voltage_req_sent_timestamp = millis();
      this->protocol_->add_request(0xAB);
    }
  }

  if (this->temperature_sensor_ != nullptr) {
    if (this->system_state.heat_exchanger_temp != this->temperature_sensor_->state) {
      this->temperature_sensor_->publish_state(this->system_state.heat_exchanger_temp);
    }
  }
  
  if (this->voltage_sensor_ != nullptr) {
    if (this->system_state.voltage != this->voltage_sensor_->state) {
      this->voltage_sensor_->publish_state(this->system_state.voltage);
    }
  }
  
  if (this->power_sensor_ != nullptr) {
    if (this->system_state.heating_power != this->power_sensor_->state) {
      this->power_sensor_->publish_state(this->system_state.heating_power);
    }
  }
  
  if (this->mode_sensor_ != nullptr) {
    if (this->system_state.mode != this->mode_sensor_->state) {
      this->mode_sensor_->publish_state(this->system_state.mode);
    }
  }
  
  if (this->alpine_sensor_ != nullptr) {
    if (this->system_state.alpine != this->alpine_sensor_->state) {
      this->alpine_sensor_->publish_state(this->system_state.alpine);
    }
  }
  
  if (this->fan_sensor_ != nullptr) {
    if (this->system_state.fan != this->fan_sensor_->state) {
      this->fan_sensor_->publish_state(this->system_state.fan);
    }
  }
  
  if (this->pump_sensor_ != nullptr) {
    if (this->system_state.pump != this->pump_sensor_->state) {
      this->pump_sensor_->publish_state(this->system_state.pump);
    }
  }
  
  if (this->spark_plug_sensor_ != nullptr) {
    if (this->system_state.spark_plug != this->spark_plug_sensor_->state) {
      this->spark_plug_sensor_->publish_state(this->system_state.spark_plug);
    }
  }

  if (this->cooling_sensor_ != nullptr) {
    if (this->system_state.cooling != this->cooling_sensor_->state) {
      this->cooling_sensor_->publish_state(this->system_state.cooling);
    }
  }
  
  if (this->power_switch_ != nullptr) {
    if (this->system_state.on != this->power_switch_->state) {
      this->power_switch_->publish_state(this->system_state.on);
    }
  }
  
  if (this->mode_switch_ != nullptr) {
    if (this->system_state.mode != this->mode_switch_->state) {
      this->mode_switch_->publish_state(this->system_state.mode);
    }
  }
  
  if (this->alpine_switch_ != nullptr) {
    if (this->system_state.alpine != this->alpine_switch_->state) {
      this->alpine_switch_->publish_state(this->system_state.alpine);
    }
  }

  // switches
  if (this->power_switch_ != nullptr) {
    if (this->system_state.on != this->power_switch_->state) {
      this->power_switch_->publish_state(this->system_state.on);
    }
  }
  if (this->mode_switch_ != nullptr) {
    if (this->system_state.mode != this->mode_switch_->state) {
      this->mode_switch_->publish_state(this->system_state.mode);
    }
  }
  if (this->alpine_switch_ != nullptr) {
    if (this->system_state.alpine != this->alpine_switch_->state) {
      this->alpine_switch_->publish_state(this->system_state.alpine);
    }
  }
}

void DieselHeater::enable() {
  this->protocol_->start();
}

void DieselHeater::disable() {
  ESP_LOGD(TAG, "Disabling DH-PRotocol");
  this->protocol_->stop();
}

IRAM_ATTR void DieselHeater::on_request_callback(uint8_t data) {
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->on_request_received(data);
  }
}

IRAM_ATTR void DieselHeater::on_response_callback(uint16_t data) {
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->on_response_received(data);
  }
}

bool DieselHeater::set_power_switch_state(bool state) {
  if (this->system_state.on != state) {
    ESP_LOGD(TAG, "Setting power switch state to %d", state);
    this->system_state.on = state;
    this->protocol_->add_request(0xA2);
    return true;
  }
  return false;
}

bool DieselHeater::set_mode_switch_state(bool state) {
  if (this->system_state.mode != state) {
    this->system_state.mode = state;
    this->protocol_->add_request(0xA1);
    return true;
  }
  return false;
}

bool DieselHeater::set_alpine_switch_state(bool state) {
  if (this->system_state.alpine != state) {
    this->system_state.alpine = state;
    this->protocol_->add_request(0xAE);
    return true;
  }
  return false;
}

void DieselHeater::set_power_up_button_clicked() {
  this->system_state.adjust_heating_power_up();
  this->protocol_->add_request(0xA3);
}

void DieselHeater::set_power_down_button_clicked() {
  this->system_state.adjust_heating_power_down();
  this->protocol_->add_request(0xA4);
}

void DieselHeater::on_request_received(uint8_t data) {
  this->request = data;
  parse_request(data, system_state);
  // data_pending = true;
}

void DieselHeater::on_response_received(uint16_t data) {
  this->response = data;
  parse_response(data, system_state);
  data_pending = true;
}

}  // namespace diesel_heater
}  // namespace esphome
