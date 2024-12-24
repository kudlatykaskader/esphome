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
    start_data_read(23);
  }
  if (this->temperature_sensor_ != nullptr) {
    if (this->system_state.heat_exchanger_temp != this->temperature_sensor_->state) {
      this->temperature_sensor_->publish_state(this->system_state.heat_exchanger_temp);
      ESP_LOGD(TAG, "Heat exchanger temp: %d", this->system_state.heat_exchanger_temp);
    }
  }
  if (this->voltage_sensor_ != nullptr) {
    if (this->system_state.voltage != this->voltage_sensor_->state) {
      this->voltage_sensor_->publish_state(this->system_state.voltage);
      ESP_LOGD(TAG, "Voltage: %d", this->system_state.voltage);
    }
  }
  if (this->power_sensor_ != nullptr) {
    if (this->system_state.heating_power != this->power_sensor_->state) {
      this->power_sensor_->publish_state(this->system_state.heating_power);
      ESP_LOGD(TAG, "Heating power: %d", this->system_state.heating_power);
    }
  }
  if (this->mode_sensor_ != nullptr) {
    if (this->system_state.mode != this->mode_sensor_->state) {
      this->mode_sensor_->publish_state(this->system_state.mode);
      ESP_LOGD(TAG, "Mode: %d", this->system_state.mode);
    }
  }
  if (this->alpine_sensor_ != nullptr) {
    if (this->system_state.alpine != this->alpine_sensor_->state) {
      this->alpine_sensor_->publish_state(this->system_state.alpine);
      ESP_LOGD(TAG, "Alpine: %d", this->system_state.alpine);
    }
  }
  if (this->fan_sensor_ != nullptr) {
    if (this->system_state.fan != this->fan_sensor_->state) {
      this->fan_sensor_->publish_state(this->system_state.fan);
      ESP_LOGD(TAG, "Fan: %d", this->system_state.fan);
    }
  }
  if (this->pump_sensor_ != nullptr) {
    if (this->system_state.pump != this->pump_sensor_->state) {
      this->pump_sensor_->publish_state(this->system_state.pump);
      ESP_LOGD(TAG, "Pump: %d", this->system_state.pump);
    }
  }
  if (this->spark_plug_sensor_ != nullptr) {
    if (this->system_state.spark_plug != this->spark_plug_sensor_->state) {
      this->spark_plug_sensor_->publish_state(this->system_state.spark_plug);
      ESP_LOGD(TAG, "Spark plug: %d", this->system_state.spark_plug);
    }
  }
  if (this->power_switch_ != nullptr) {
    if (this->system_state.on != this->power_switch_->state) {
      this->power_switch_->publish_state(this->system_state.on);
      ESP_LOGD(TAG, "Power: %d", this->system_state.on);
    }
  }
  if (this->mode_switch_ != nullptr) {
    if (this->system_state.mode != this->mode_switch_->state) {
      this->mode_switch_->publish_state(this->system_state.mode);
      ESP_LOGD(TAG, "Mode: %d", this->system_state.mode);
    }
  }
  if (this->alpine_switch_ != nullptr) {
    if (this->system_state.alpine != this->alpine_switch_->state) {
      this->alpine_switch_->publish_state(this->system_state.alpine);
      ESP_LOGD(TAG, "Alpine: %d", this->system_state.alpine);
    }
  }

  // switches
  if (this->power_switch_ != nullptr) {
    if (this->system_state.on != this->power_switch_->state) {
      this->power_switch_->publish_state(this->system_state.on);
      ESP_LOGD(TAG, "Power: %d", this->system_state.on);
    }
  }
  if (this->mode_switch_ != nullptr) {
    if (this->system_state.mode != this->mode_switch_->state) {
      this->mode_switch_->publish_state(this->system_state.mode);
      ESP_LOGD(TAG, "Mode: %d", this->system_state.mode);
    }
  }
  if (this->alpine_switch_ != nullptr) {
    if (this->system_state.alpine != this->alpine_switch_->state) {
      this->alpine_switch_->publish_state(this->system_state.alpine);
      ESP_LOGD(TAG, "Alpine: %d", this->system_state.alpine);
    }
  }

}

bool DieselHeater::set_power_switch_state(bool state) {
  if (this->system_state.on != state) {
    this->system_state.on = state;
    // this->request_queue_.push(RequestType::POWER_TOGGLE);
    return true;
  }
  return false;
}

bool DieselHeater::set_mode_switch_state(bool state) {
  if (this->system_state.mode != state) {
    this->system_state.mode = state;
    // this->request_queue_.push(RequestType::MODE_TOGGLE);
    return true;
  }
  return false;
}

bool DieselHeater::set_alpine_switch_state(bool state) {
  if (this->system_state.alpine != state) {
    this->system_state.alpine = state;
    // this->request_queue_.push(RequestType::ALPINE_TOGGLE);
    return true;
  }
  return false;
}

void DieselHeater::set_power_up_button_clicked() {
  this->system_state.adjust_heating_power_up();
  // this->request_queue_.push(RequestType::POWER_UP);
}

void DieselHeater::set_power_down_button_clicked() {
  this->system_state.adjust_heating_power_down();
  // this->request_queue_.push(RequestType::POWER_DOWN);
}

void DieselHeater::start_data_read(uint8_t bits) {
  sm_.set_state(ReadState::F_REQ_WAIT_F_EDGE);
  sm_.set_bits_to_read(bits);
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
    case ReadState::F_RESP_WAIT_F_EDGE:
      platform_.detach_pin_interrupt();
      sm_.set_state(ReadState::F_RESP_WAIT_R_EDGE);
      platform_.attach_pin_interrupt(this->data_pin_, true /* rising edge */);
      break;

    case ReadState::F_RESP_WAIT_R_EDGE:
      platform_.detach_pin_interrupt();
      platform_.start_timer(TIME_PERIOD_4040us / 2);
      sm_.set_state(ReadState::F_RESP_READ);
      sm_.set_bits_to_read(47);
      sm_.set_bit_index(0);
      break;

    case ReadState::F_RESP_WAIT_END:
      platform_.detach_pin_interrupt();
      sm_.set_state(ReadState::F_REQ_IDLE);
      // if (this->request_queue_.size() > 0) {
      //   RequestType req = this->request_queue_.front();
      //   this->request_queue_.pop();
      //   Response resp = generate_response(req, system_state, 0);
      //   for (uint8_t i = 0; i < 48; i++) {
      //     current_response[i] = resp.data & (1 << i);
      //   }
      //   sm_.set_state(ReadState::F_REQ_PRE_WAIT);
      //   platform_.start_timer(TIME_PERIOD_1700us);
      //   // pull data line high for 1700us
      // }
      break;

    case ReadState::F_REQ_WAIT_END:
      toggle_debug_pin(this->debug_pin_2_, 5);
      if (op_mode_ == Mode::SIMULATOR) {
        // Simulate response
        sm_.set_state(ReadState::F_RESP_PRE_WAIT);
        platform_.start_timer(TIME_PERIOD_1700us);
      } else if (op_mode_ == Mode::ADAPTER) {
        sm_.set_state(ReadState::F_RESP_WAIT_F_EDGE);
        platform_.attach_pin_interrupt(this->data_pin_, false /* falling edge */);
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
      if (this->data_pin_ != nullptr)
        this->data_pin_->pin_mode(gpio::FLAG_OUTPUT);
      digitalWrite(this->data_pin_->get_pin(), LOW);
      platform_.start_timer(TIME_PERIOD_30ms);
      // Done reading request
      this->on_request_received();
      break;

    case ReadState::F_RESP_WRITE:
      // Write response bits
      platform_.start_timer(TIME_PERIOD_4040us);
      digitalWrite(this->data_pin_->get_pin(), this->current_response[sm_.current_bit_index()]);
      sm_.increment_bit_index();
      if (sm_.current_bit_index() > 48) {
        // Done sending response
        sm_.reset();
        // sm_.set_state(ReadState::F_REQ_IDLE);
        this->start_data_read(23);
        if (this->data_pin_ != nullptr)
          this->data_pin_->pin_mode(gpio::FLAG_INPUT);
        platform_.stop_timer();
      }
      break;

    case ReadState::F_RESP_WAIT_F_EDGE:
      platform_.detach_pin_interrupt();
      sm_.set_state(ReadState::F_RESP_WAIT_R_EDGE);
      platform_.attach_pin_interrupt(this->data_pin_, true /* rising edge */);
      break;

    case ReadState::F_RESP_READ:
      this->toggle_debug_pin(this->debug_pin_1_, 5);
      current_response[sm_.current_bit_index()] = (uint8_t) digitalRead(this->data_pin_->get_pin());
      // Store bit if needed (this would be in a buffer)
      sm_.increment_bit_index();

      if (sm_.current_bit_index() > sm_.bits_to_read()) {
        sm_.set_state(ReadState::F_RESP_WAIT_END);
        platform_.stop_timer();
        platform_.attach_pin_interrupt(this->data_pin_, true /* rising */);

        this->on_response_received();
        // if request_queue_ is not empty, generate request and read response
        // if (this->request_queue_.size() > 0) {
          // current_request = generate_request(this->request_queue_.front());
          // this->request_queue_.pop();
          // sm_.set_state(ReadState::F_REQ_PRE_WAIT);
          // platform_.start_timer(TIME_PERIOD_1700us);
        // }
      } else {
        // Continue reading bits at 4.04ms intervals
        timer1_write(TIME_PERIOD_4040us);
      }
      break;
    case ReadState::F_REQ_PRE_WAIT:
      sm_.set_state(ReadState::F_REQ_WRITE);
      platform_.start_timer(TIME_PERIOD_30ms);
      if (this->data_pin_ != nullptr)
        this->data_pin_->pin_mode(gpio::FLAG_OUTPUT);
      digitalWrite(this->data_pin_->get_pin(), LOW);

      break;
    case ReadState::F_REQ_WRITE:
      platform_.start_timer(TIME_PERIOD_4040us);
      sm_.set_bit_index(0);
      digitalWrite(this->data_pin_->get_pin(), this->current_response[sm_.current_bit_index()]);
      sm_.increment_bit_index();
      if (sm_.current_bit_index() > 24) {
        // Done sending response
        sm_.reset();
        // sm_.set_state(ReadState::F_REQ_IDLE);
        // this->start_data_read(48);
        if (this->data_pin_ != nullptr)
          this->data_pin_->pin_mode(gpio::FLAG_INPUT);
        platform_.stop_timer();
      }

    default:
      break;
  }
}

void DieselHeater::on_request_received() {
  handle_request(current_request, current_response, system_state);
}

void DieselHeater::on_response_received() {
  handle_response(current_response, system_state);
}

void DieselHeater::toggle_debug_pin(InternalGPIOPin *pin, uint32_t delay_us) {
  if (pin == nullptr)
    return;
  pin->digital_write(HIGH);
  delayMicroseconds(delay_us);
  pin->digital_write(LOW);
}

}  // namespace diesel_heater
}  // namespace esphome
