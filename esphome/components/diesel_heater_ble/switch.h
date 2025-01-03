#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/log.h"
#include "heater.h"
#include "messages.h"

namespace esphome {
namespace diesel_heater_ble {

class PowerSwitch : public switch_::Switch, public Parented<DieselHeaterBLE> {
 public:
  PowerSwitch() = default;

 protected:
  void write_state(bool state) override {
    ESP_LOGD("diesel_heater_ble", "Setting power state to: %d", state);
    this->parent_->sent_request(SetPowerRequest(state).toBytes());
  }
};

}  // namespace diesel_heater
}  // namespace esphome