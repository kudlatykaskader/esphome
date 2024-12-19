#pragma once

#include "esphome/core/hal.h"

namespace esphome {
namespace diesel_heater {

// Forward-declare the main class
class DieselHeater;

class PlatformInterface {
 public:
  explicit PlatformInterface(DieselHeater *parent) : parent_(parent) {}
  void start_timer(uint32_t us);
  void stop_timer();
  void attach_pin_interrupt(InternalGPIOPin *pin, bool rising);
  void detach_pin_interrupt();

  static void IRAM_ATTR on_pin_isr();
  static void IRAM_ATTR on_timer_isr();

 private:
  DieselHeater *parent_;
};

}  // namespace diesel_heater
}  // namespace esphome
