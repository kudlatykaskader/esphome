#include "switch.h"

namespace esphome {
namespace diesel_heater {

void PowerSwitch::write_state(bool state) {
  if (this->parent_->get_power_switch() != state) {
    this->parent_->set_power_switch(state);
  }
  this->publish_state(state);
}

void ModeSwitch::write_state(bool state) {
  if (this->parent_->get_mode_switch() != state) {
    this->parent_->set_mode_switch(state);
  }
  this->publish_state(state);
}

void AlpineSwitch::write_state(bool state) {
  if (this->parent_->get_alpine_switch() != state) {
    this->parent_->set_alpine_switch(state);
  }
  this->publish_state(state);
}

}  // namespace haier
}  // namespace esphome
