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
  LOG_PIN("  Debug Pin 1: ", this->debug_pin_1_);
  LOG_PIN("  Debug Pin 2: ", this->debug_pin_2_);
}

void DieselHeater::setup() {
  DieselHeater::instance_ = this;

  // Setup pins
  if (this->data_pin_ != nullptr) {
    this->data_pin_->setup();
    this->data_pin_->pin_mode(gpio::FLAG_INPUT);
  }
  
  if (this->debug_pin_1_ != nullptr) {
    this->debug_pin_1_->setup();
    this->debug_pin_1_->pin_mode(gpio::FLAG_OUTPUT);
  }

  if (this->debug_pin_2_ != nullptr) {
    this->debug_pin_2_->setup();
    this->debug_pin_2_->pin_mode(gpio::FLAG_OUTPUT);
  }

  // Initially, we are idle
  sm_.reset();
}

void DieselHeater::loop() {
  // If we are idle, start waiting for a frame
  if (sm_.current_state() == ReadState::F_REQ_IDLE) {
    start_data_read();
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
}

void DieselHeater::start_data_read() {
  sm_.set_state(ReadState::F_REQ_WAIT_F_EDGE);
  sm_.set_bits_to_read(23);
  sm_.set_bit_index(0);
  // Wait for falling edge
  platform_.attach_pin_interrupt(this->data_pin_, false /* falling edge */);
}

void DieselHeater::on_pin_isr() {
  uint32_t now = micros();
  switch (sm_.current_state()) {
    case ReadState::F_REQ_WAIT_F_EDGE:
      this->toggle_debug_pin(this->debug_pin_2_, 5);
      sm_.on_falling_edge_detected(now);
      // Now wait for rising edge
      platform_.detach_pin_interrupt();
      platform_.attach_pin_interrupt(this->data_pin_, true /* rising edge */);
      break;

    case ReadState::F_REQ_WAIT_R_EDGE: {
      platform_.detach_pin_interrupt();
      bool valid_start = sm_.on_rising_edge_detected(now);
      if (valid_start) {
        // Start reading bits at 4.04ms intervals
        platform_.start_timer(TIME_PERIOD_4040us / 2);
        sm_.set_state(ReadState::F_REQ_READ);
      } else {
        sm_.set_state(ReadState::F_REQ_IDLE);
      }
      break;
    }

    case ReadState::F_REQ_WAIT_END:
      toggle_debug_pin(this->debug_pin_2_, 5);
      // Request ended, decide next step based on op_mode
      if (op_mode_ == OperatingMode::MODE_HEATER) {
        sm_.set_state(ReadState::F_RESP_PRE_WAIT);
        // Prepare to send response
        // Wait ~1.7ms and then send idle_response_
        // platform_.stop_timer();
        // Reuse timer for response pre-wait
        platform_.start_timer(TIME_PERIOD_1700us);
      } else if (op_mode_ == OperatingMode::MODE_CONTROLLER_SHARED) {
        // If needed, handle shared mode reading
      } else {
        // Exclusive controller mode handling
      }
      break;

    default:
      break;
  }
}

void DieselHeater::on_timer_isr() {
  switch (sm_.current_state()) {
    case ReadState::F_REQ_READ: {
      // Read next bit
      this->toggle_debug_pin(this->debug_pin_1_, 5);
      uint8_t bit = (uint8_t) digitalRead(this->data_pin_->get_pin());
      current_request[sm_.current_bit_index()] = bit;
      // Store bit if needed (this would be in a buffer)
      sm_.increment_bit_index();
      
      if (sm_.current_bit_index() > sm_.bits_to_read()) {
        sm_.set_state(ReadState::F_REQ_WAIT_END);
        platform_.stop_timer();
        platform_.attach_pin_interrupt(this->data_pin_, true /* rising */);
      } else {
        // Continue reading bits at 4.04ms intervals
        timer1_write(TIME_PERIOD_4040us);
      }
      break;
    }

    case ReadState::F_RESP_PRE_WAIT:
      // Switch to write mode
      sm_.set_bit_index(0);
      sm_.set_state(ReadState::F_RESP_WRITE);
      if (this->data_pin_ != nullptr) this->data_pin_->pin_mode(gpio::FLAG_OUTPUT);
      digitalWrite(this->data_pin_->get_pin(), LOW);
      platform_.start_timer(TIME_PERIOD_30ms);
      // Done reading request
      handle_request(current_request, current_response, system_state);
      break;

    case ReadState::F_RESP_WRITE: {
      // Write response bits
      platform_.start_timer(TIME_PERIOD_4040us);
      uint8_t idx = sm_.current_bit_index();
      digitalWrite(this->data_pin_->get_pin(), this->current_response[idx]);
      sm_.increment_bit_index();
      if (sm_.current_bit_index() > 48) {
        // Done sending response
        sm_.reset();
        // sm_.set_state(ReadState::F_REQ_IDLE);
        this->start_data_read();
        if (this->data_pin_ != nullptr) this->data_pin_->pin_mode(gpio::FLAG_INPUT);
        platform_.stop_timer();

      }
      break;
    }

    default:
      break;
  }
}

void DieselHeater::toggle_debug_pin(InternalGPIOPin *pin, uint32_t delay_us) {
  if (pin == nullptr) return;
  pin->digital_write(HIGH);
  delayMicroseconds(delay_us);
  pin->digital_write(LOW);
}

void DieselHeater::decode_raw_request() {
  // Future decoding logic here
}

}  // namespace diesel_heater
}  // namespace esphome
