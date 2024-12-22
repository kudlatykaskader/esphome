#pragma once

#include "esphome/components/switch/switch.h"
#include "diesel_heater.h"

namespace esphome {
namespace diesel_heater {

class PowerSwitch : public switch_::Switch, public Parented<DieselHeater> {
 public:
  PowerSwitch() = default;

 protected:
  void write_state(bool state) override;
};

class ModeSwitch : public switch_::Switch, public Parented<DieselHeater> {
 public:
  ModeSwitch() = default;

 protected:
  void write_state(bool state) override;
};

class AlpineSwitch : public switch_::Switch, public Parented<DieselHeater> {
 public:
  AlpineSwitch() = default;

 protected:
  void write_state(bool state) override;
};

}  // namespace diesel_heater
}  // namespace esphome
