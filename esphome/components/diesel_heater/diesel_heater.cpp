#include "diesel_heater.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include <Arduino.h>

namespace esphome {
namespace diesel_heater {

static const char *const TAG = "heater";

// Static instance pointer for ISR
DieselHeater *DieselHeater::instance_ = nullptr;

void DieselHeater::dump_config() {
  ESP_LOGCONFIG(TAG, "DieselHeater:");
  LOG_PIN("  Data Pin: ", this->data_pin_);
  LOG_PIN("  Debug Pin 1: ", this->debug_pin_1_);
  LOG_PIN("  Debug Pin 2: ", this->debug_pin_2_);
}

void DieselHeater::setup() {
  DieselHeater::instance_ = this;
  
  if (this->data_pin_ != nullptr) {
    this->data_pin_->setup();
  }

  if (this->debug_pin_1_ != nullptr) {
    this->debug_pin_1_->setup();
    this->debug_pin_1_->pin_mode(gpio::FLAG_OUTPUT);
  }

  if (this->debug_pin_2_ != nullptr) {
    this->debug_pin_2_->setup();
    this->debug_pin_2_->pin_mode(gpio::FLAG_OUTPUT);
  }

  if (this->data_pin_ != nullptr) {
    this->data_pin_->setup();
    this->data_pin_->pin_mode(gpio::FLAG_INPUT);
  }

}

void DieselHeater::loop() {
  switch (this->current_state_) {
    case ReadState::F_REQ_IDLE:
      this->start_data_read();
      break;
    default:
      break;
  }
}

void DieselHeater::start_data_read() {
  this->current_state_ = ReadState::F_REQ_WAIT_F_EDGE;
  if (this->data_pin_ != nullptr) this->data_pin_->pin_mode(gpio::FLAG_INPUT);
  this->bits_to_read_ = 23;
  this->current_bit_index = 0;
  attachInterrupt(digitalPinToInterrupt(this->data_pin_->get_pin()), static_on_pin_isr, FALLING);
}

void IRAM_ATTR DieselHeater::static_on_pin_isr() {
  DieselHeater::instance_->on_pin_isr();
}

void IRAM_ATTR DieselHeater::static_on_timer_isr() {
  DieselHeater::instance_->on_timer_isr();
}

// static interrupt handlers
void IRAM_ATTR DieselHeater::on_pin_isr() {
  uint32_t duration = 0;
  switch (this->current_state_) {
    case ReadState::F_REQ_WAIT_F_EDGE:
      this->time_frame_prefix_started = micros();
      this->current_state_ = ReadState::F_REQ_WAIT_R_EDGE;
      attachInterrupt(digitalPinToInterrupt(this->data_pin_->get_pin()), static_on_pin_isr, RISING);
      break;
    case ReadState::F_REQ_WAIT_R_EDGE:
      detachInterrupt(digitalPinToInterrupt(this->data_pin_->get_pin())); 
      duration = micros() - this->time_frame_prefix_started;
      if (duration > 29000 && duration < 31000) {
          this->start_timer(TIME_PERIOD_4040us/2, static_on_timer_isr);
          this->current_state_ = ReadState::F_REQ_READ;
          break;
      }
      this->current_state_ = ReadState::F_REQ_IDLE;
      break;
    case ReadState::F_REQ_WAIT_END:
      // At this point a request end has been detected. 
      // IF operating mode is heater, a valid response must be sent
      // IF operating mode is shared, we start reading response from heater 
      if (this->op_mode == OperatingMode::MODE_HEATER) {
        this->current_state_ = ReadState::F_RESP_PRE_WAIT;
        timer1_attachInterrupt(static_on_timer_isr);
        this->current_bit_index = 0;
        timer1_write(TIME_PERIOD_1700us);
      } else if (this->op_mode == OperatingMode::MODE_CONTROLLER_SHARED) {
        // this->current_state_ = ReadState::F_RESP_PRE_WAIT;
      } else {

      }
    default:
      break;
  }
}

void IRAM_ATTR DieselHeater::on_timer_isr() {
  uint8_t bit = 0;
  switch (this->current_state_)
  {
  case ReadState::F_REQ_READ:
    timer1_write(TIME_PERIOD_4040us);
    bit = (uint8_t)digitalRead(this->data_pin_->get_pin());
    if (this->current_bit_index >= this->bits_to_read_) {
      this->current_state_ = ReadState::F_REQ_WAIT_END;
      timer1_detachInterrupt();
      attachInterrupt(digitalPinToInterrupt(this->data_pin_->get_pin()), static_on_pin_isr, RISING);
    } else {
      timer1_write(TIME_PERIOD_4040us);
    }

    this->current_bit_index++;
    break;
  case ReadState::F_RESP_PRE_WAIT:
    this->current_state_ = ReadState::F_RESP_WRITE;
    if (this->data_pin_ != nullptr) this->data_pin_->pin_mode(gpio::FLAG_OUTPUT);
    digitalWrite(this->data_pin_->get_pin(), LOW);
    timer1_write(TIME_PERIOD_30ms);
    break;
  case ReadState::F_RESP_WRITE:
    timer1_write(TIME_PERIOD_4040us);
    digitalWrite(this->data_pin_->get_pin(), this->idle_response[this->current_bit_index]);
    if (this->current_bit_index >= 48) {
      this->current_state_ = ReadState::F_REQ_IDLE;
      timer1_detachInterrupt();
    }
    this->current_bit_index++;
    break;
  default:
    break;
  }
}

void DieselHeater::decode_raw_request() {
  // this->raw_request_buffer >> 1;
  // for(int i = 0; i < this->bits_to_read_/3; i++) {
  //   decoded_request_buffer[bits_to_read_/3-i] = this->raw_request_buffer & 0x01 
  // }
}

void DieselHeater::start_timer(uint32_t us, void (*fn)()) {
#ifdef USE_ESP8266
  timer1_attachInterrupt(fn);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(us);
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  timerAlarmWrite(this->timer_, SUB_BIT_US, true);
  timerAlarmEnable(this->timer_);
#endif
}

void DieselHeater::stop_timer() {
#ifdef USE_ESP8266
  timer1_detachInterrupt();
  timer1_disable();
  timer1_write(0);
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  timerAlarmDisable(this->timer_);
#endif
}

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
void IRAM_ATTR DieselHeater::on_timer_isr() {
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->data_reading_callback();
  }
}
#endif

}  // namespace diesel_heater
}  // namespace esphome
