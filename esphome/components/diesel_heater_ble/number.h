#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

#include "heater.h"
#include "messages.h"

namespace esphome {
namespace diesel_heater_ble {

class PowerLevelNumber : public number::Number, public Parented<DieselHeaterBLE> {
  public:
  PowerLevelNumber() = default;
  
  protected:
  void control(float value) override {
    if (this->parent_->get_state().runningmode == 2) {
      this->parent_->sent_request(SetRunningModeRequest(1).toBytes());
    }
    this->parent_->sent_request(SetLevelRequest(value + 1).toBytes());
  }
};

class SetTempNumber : public number::Number, public Parented<DieselHeaterBLE> {
  public:
  SetTempNumber() = default;
  
  protected:
  void control(float value) override {
    if (this->parent_->get_state().runningmode == 1) {
      this->parent_->sent_request(SetRunningModeRequest(2).toBytes());
    }
    this->parent_->sent_request(SetTemperatureRequest(value).toBytes());
  }
};

}  // namespace diesel_heater_ble
}  // namespace esphome