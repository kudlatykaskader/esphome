#include "button.h"

namespace esphome {
namespace diesel_heater {

void PowerUpButton::press_action() {
  this->parent_->increase_power();
}

void PowerDownButton::press_action() {
  this->parent_->decrease_power();
}

}  // namespace diesel_heater
}  // namespace esphome
