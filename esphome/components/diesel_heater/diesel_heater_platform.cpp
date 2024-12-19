#include "diesel_heater_platform.h"
#include "diesel_heater.h"

#ifdef USE_ESP8266
#include <core_esp8266_waveform.h>
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include <esp32-hal-timer.h>
#endif

namespace esphome {
namespace diesel_heater {

void PlatformInterface::attach_pin_interrupt(InternalGPIOPin *pin, bool rising) {
  attachInterrupt(digitalPinToInterrupt(pin->get_pin()), on_pin_isr, rising ? RISING : FALLING);
}

void PlatformInterface::detach_pin_interrupt() {
  detachInterrupt(-1);
}

void PlatformInterface::start_timer(uint32_t us) {
#ifdef USE_ESP8266
  timer1_attachInterrupt(on_timer_isr);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(us);
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  // For ESP32, you'd store a reference to the timer in the parent class
  // and configure it here
  // timerAlarmWrite(this->parent_->timer_, us, true);
  // timerAlarmEnable(this->parent_->timer_);
#endif
}

void PlatformInterface::stop_timer() {
#ifdef USE_ESP8266
  timer1_detachInterrupt();
  timer1_write(0);
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  // For ESP32
  // timerAlarmDisable(this->parent_->timer_);
#endif
}

void IRAM_ATTR PlatformInterface::on_pin_isr() {
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->on_pin_isr();
  }
}

void IRAM_ATTR PlatformInterface::on_timer_isr() {
  if (DieselHeater::instance_ != nullptr) {
    DieselHeater::instance_->on_timer_isr();
  }
}

}  // namespace diesel_heater
}  // namespace esphome
