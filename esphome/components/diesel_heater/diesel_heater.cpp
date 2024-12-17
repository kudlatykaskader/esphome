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

void DieselHeater::setup() {
  if (this->data_pin_ != nullptr) {
    this->data_pin_->setup();
    this->data_pin_->pin_mode(gpio::FLAG_INPUT);

    DieselHeater::instance_ = this;
    // Attach interrupt to handle line changes
    attachInterrupt(digitalPinToInterrupt(this->data_pin_->get_pin()), handle_line_change_isr, CHANGE);
    this->last_edge_time_ = micros();
  }

  if (this->debug_pin_ != nullptr) {
    this->debug_pin_->setup();
    this->debug_pin_->pin_mode(gpio::FLAG_OUTPUT);
  }

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  // Initialize timer for ESP32, but don't start yet
  this->timer_ = timerBegin(0, 80, true); // 1 tick = 1µs at 80MHz/80 prescale
  timerAttachInterrupt(this->timer_, &DieselHeater::on_timer_isr, true);
#endif
}

void DieselHeater::loop() {
  // If frame received, handle it
  if (this->frame_received_) {
    this->frame_received_ = false;
    ESP_LOGD(TAG, "Frame received: 0x%02X", this->received_byte_);

    // Reset state
    this->current_state_ = ReadState::WAIT_FOR_START;
    this->bit_count_ = 0;
    this->stop_timing();
  }
}

void DieselHeater::dump_config() {
  ESP_LOGCONFIG(TAG, "DieselHeater:");
  LOG_PIN("  Data Pin: ", this->data_pin_);
  LOG_PIN("  Debug Pin: ", this->debug_pin_);
}

void IRAM_ATTR DieselHeater::handle_line_change_isr() {
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->handle_line_change();
  }
}

void IRAM_ATTR DieselHeater::handle_line_change() {
  int level = digitalRead(this->data_pin_->get_pin());
  uint32_t now = micros();
  uint32_t duration = now - this->last_edge_time_;
  this->last_edge_time_ = now;

  if (this->current_state_ == ReadState::WAIT_FOR_START) {
    // Detect ~30ms low -> then line goes HIGH
    if (level == HIGH) {
      if (duration > 20000 && duration < 40000) {
        ESP_LOGD(TAG, "Start condition detected, time passed: %d", duration);
        // Start condition detected
        this->current_state_ = ReadState::READING_BITS;
        this->bit_count_ = 0;
        this->sub_bit_count_ = 0;
        this->current_byte_ = 0;
        delayMicroseconds(SUB_BIT_US / 2); // Skip first half of first sub-bit
        this->start_timing();
      }
    }
  }
}

void DieselHeater::decode_sub_bits() {
  // '1' bit = 110
  // '0' bit = 100
  if (sub_bit_buffer_[0] == 1 && sub_bit_buffer_[1] == 1 && sub_bit_buffer_[2] == 0) {
    this->current_byte_ = (this->current_byte_ << 1) | 1;
  } else if (sub_bit_buffer_[0] == 1 && sub_bit_buffer_[1] == 0 && sub_bit_buffer_[2] == 0) {
    this->current_byte_ = (this->current_byte_ << 1) | 0;
  } else {
    // Pattern error handling if needed
  }

  this->bit_count_++;
  this->sub_bit_count_ = 0;

  if (this->bit_count_ == 8) {
    // Full byte received
    this->received_byte_ = this->current_byte_;
    this->frame_received_ = true;
    this->current_state_ = ReadState::FRAME_COMPLETE;
    this->bit_count_ = 0;

  }
}

void DieselHeater::start_timing() {
#ifdef USE_ESP8266
  // On ESP8266 use waveform to get ~4.04ms callback
  // We'll just split SUB_BIT_US evenly: 2020us high, 2020us low = 4040us total

  // First event must happen after 2020us
  startWaveform(DUMMY_WAVEFORM_PIN, SUB_BIT_US, SUB_BIT_US);
  setTimer1Callback(&waveform_callback);
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  timerAlarmWrite(this->timer_, SUB_BIT_US, true);
  timerAlarmEnable(this->timer_);
#endif
}

void DieselHeater::stop_timing() {
#ifdef USE_ESP8266
  stopWaveform(DUMMY_WAVEFORM_PIN);
  setTimer1Callback(nullptr);
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  timerAlarmDisable(this->timer_);
#endif
}

#ifdef USE_ESP8266
uint32_t IRAM_ATTR DieselHeater::waveform_callback() {
  // Each callback ~4.04ms
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->on_timer_callback();

  }
  if (DieselHeater::instance_->frame_received_) {
    return 0;
  } else {
    return microsecondsToClockCycles(SUB_BIT_US);
  }
}
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
void IRAM_ATTR DieselHeater::on_timer_isr() {
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->on_timer_callback();
  }
}
#endif

void IRAM_ATTR DieselHeater::on_timer_callback() {
  this->toggle_debug_pin();
  if (this->current_state_ == ReadState::READING_BITS) {
    uint8_t level = (uint8_t)digitalRead(this->data_pin_->get_pin());
    this->sub_bit_buffer_[this->sub_bit_count_] = level;
    this->sub_bit_count_++;

    if (this->sub_bit_count_ == 3) {
      this->decode_sub_bits();
    }
  }
}

}  // namespace heater
}  // namespace esphome
