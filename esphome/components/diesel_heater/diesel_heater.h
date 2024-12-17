#pragma once

#include <cinttypes>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/time.h"
#endif

#ifdef USE_ESP8266
#include <core_esp8266_waveform.h>
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include <esp32-hal-timer.h>
#endif

namespace esphome {
namespace diesel_heater {

enum class ReadState {
  WAIT_FOR_START,
  READING_BITS,
  FRAME_COMPLETE
};

class DieselHeater : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_data_pin(InternalGPIOPin *data_pin) { 
    this->data_pin_ = data_pin;
  }

  void set_debug_pin(InternalGPIOPin *debug_pin) {
    this->debug_pin_ = debug_pin;
  }

  void toggle_debug_pin() {
    if (this->debug_pin_ != nullptr) {
      this->debug_pin_->digital_write(!this->debug_pin_->digital_read());
    }
  }

 protected:
  InternalGPIOPin *data_pin_{nullptr};
  InternalGPIOPin *debug_pin_{nullptr};

  // Protocol reading variables
  volatile ReadState current_state_{ReadState::WAIT_FOR_START};
  volatile uint32_t last_edge_time_{0};
  volatile uint32_t last_read_time_{micros()};
  volatile uint8_t bit_count_{0};
  volatile uint8_t sub_bit_count_{0};
  volatile uint8_t current_byte_{0};
  volatile uint8_t received_byte_{0};
  volatile bool frame_received_{false};

  // Timing constants
  static const uint32_t SUB_BIT_US = 4040; // ~4.04 ms per sub-bit
  // We'll look for ~30ms low to detect start condition
  // 8 bits * 3 sub-bits/bit * 4.04ms/sub-bit ~ 96.96ms total reading time

#ifdef USE_ESP8266
  static const uint8_t DUMMY_WAVEFORM_PIN = 16; // Unused pin for waveform
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  hw_timer_t *timer_ = nullptr;
#endif

  volatile uint8_t sub_bit_buffer_[3];

  // ISR handling
  static DieselHeater *instance_;
  static void IRAM_ATTR handle_line_change_isr();
  void IRAM_ATTR handle_line_change();

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  static void IRAM_ATTR on_timer_isr();
  void IRAM_ATTR on_timer_callback();
#endif

#ifdef USE_ESP8266
  // Waveform callback for ESP8266
  static uint32_t waveform_callback() IRAM_ATTR;
  void IRAM_ATTR on_timer_callback();
#endif

  void start_timing();
  void stop_timing();

  void decode_sub_bits();

};

}  // namespace heater
}  // namespace esphome
