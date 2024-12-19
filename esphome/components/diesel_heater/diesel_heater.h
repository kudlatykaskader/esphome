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

// 10100110 -> 0011000000000110 1.0.7?
namespace esphome {
namespace diesel_heater {

enum class OperatingMode {
  MODE_HEATER,
  MODE_CONTROLLER_EXCLUSIVE,
  MODE_CONTROLLER_SHARED
};

enum class ReadState {
  // 8 bit dataframe from controller to heater
  F_REQ_IDLE,     // wait for a falling edge followed by 30ms low state
  F_REQ_WAIT_F_EDGE,
  F_REQ_WAIT_R_EDGE, // rising edge after 30ms low state
  F_REQ_READ,     // reading 24 (8*1<b>0) bits with 4.04ms interval
  F_REQ_WAIT_END,
  F_RESP_PRE_WAIT,
  F_RESP_WRITE
};

class DieselHeater : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_data_pin(InternalGPIOPin *data_pin) { 
    this->data_pin_ = data_pin;
  }

  void set_debug_pin_1(InternalGPIOPin *debug_pin) {
    this->debug_pin_1_ = debug_pin;
  }

  void set_debug_pin_2(InternalGPIOPin *debug_pin) {
    this->debug_pin_2_ = debug_pin;
  }

  void toggle_debug_pin_1(uint32_t delay) {
    if (this->debug_pin_1_ != nullptr) {
      this->debug_pin_1_->digital_write(HIGH);
      delayMicroseconds(delay);
      this->debug_pin_1_->digital_write(LOW);
    }
  }

  void toggle_debug_pin_2(uint32_t delay) {
    if (this->debug_pin_2_ != nullptr) {
      this->debug_pin_2_->digital_write(HIGH);
      delayMicroseconds(delay);
      this->debug_pin_2_->digital_write(LOW);
    }
  }

 protected:
  InternalGPIOPin *data_pin_{nullptr};
  InternalGPIOPin *debug_pin_1_{nullptr};
  InternalGPIOPin *debug_pin_2_{nullptr};

  void start_data_read();

  // static interrupt handlers
  static void IRAM_ATTR static_on_pin_isr();
  static void IRAM_ATTR static_on_timer_isr();

  // interrupt handlers
  void IRAM_ATTR on_pin_isr();
  void IRAM_ATTR on_timer_isr();

  // Protocol reading variables
  volatile uint8_t bits_to_read_{0};
  volatile uint8_t current_bit_index{0};
  
  volatile uint32_t time_frame_prefix_started{0};

  volatile OperatingMode op_mode{OperatingMode::MODE_HEATER};
  volatile ReadState current_state_{ReadState::F_REQ_IDLE};

  // Timing constants
  static const uint32_t TIME_PERIOD_4040us = 4040 * 5; // ~4.04 
  static const uint32_t TIME_PERIOD_30ms = 30000 * 5;
  static const uint32_t TIME_PERIOD_1700us = 1700 * 5;

  bool idle_response[48] = {1,0,0,1,1,0,1,1,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,1,0,1,1,0,1,0,0,1,0,0};
  
  #ifdef USE_ESP8266
  static const uint8_t DUMMY_WAVEFORM_PIN = 16; // Unused pin for waveform
  #endif

  // ISR handling
  static DieselHeater *instance_;

#ifdef USE_ESP8266
  // Waveform callback for ESP8266
  static uint32_t waveform_callback() IRAM_ATTR;
#endif

  void start_timer(uint32_t us, void (*fn)());
  void stop_timer();

  void decode_raw_request();

};

}  // namespace heater
}  // namespace esphome
