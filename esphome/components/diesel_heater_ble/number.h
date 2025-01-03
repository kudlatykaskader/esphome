#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

#include "heater.h"

namespace esphome {
namespace diesel_heater_ble {

class PowerLevelNumber : public number::Number, public Parented<DieselHeaterBLE> {
  public:
  PowerLevelNumber() = default;
  
  protected:
  void control(float value) override {
    this->parent_->set_power_level_action(value);
    }
};

class SetTempNumber : public number::Number, public Parented<DieselHeaterBLE> {
  public:
  SetTempNumber() = default;
  
  protected:
  void control(float value) override {
    this->parent_->set_temp_number_action(value);
  }
};

}  // namespace diesel_heater_ble
}  // namespace esphome