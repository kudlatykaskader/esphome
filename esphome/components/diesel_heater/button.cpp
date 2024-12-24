#include "button.h"

namespace esphome {
namespace diesel_heater {

void PowerUpButton::press_action() {
  this->parent_->set_power_up_button_clicked();
}

void PowerDownButton::press_action() {
  this->parent_->set_power_down_button_clicked();
}

}  // namespace diesel_heater
}  // namespace esphome
