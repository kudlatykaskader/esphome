#pragma once

#include "esphome/components/button/button.h"
#include "diesel_heater.h"

namespace esphome {
namespace diesel_heater {

class PowerUpButton : public button::Button, public Parented<DieselHeater> {
  public:
  PowerUpButton() = default;
  
  protected:
  void press_action() override;
};

class PowerDownButton : public button::Button, public Parented<DieselHeater> {
 public:
  PowerDownButton() = default;

  protected:
  void press_action() override;
};

}  // namespace diesel_heater
}  // namespace esphome
