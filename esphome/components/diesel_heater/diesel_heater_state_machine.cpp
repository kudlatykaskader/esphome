#include "diesel_heater_state_machine.h"
#include <Arduino.h>

namespace esphome {
namespace diesel_heater {

void StateMachine::reset() {
  current_state_ = ReadState::F_REQ_IDLE;
  bits_to_read_ = 0;
  current_bit_index_ = 0;
  time_frame_prefix_started_ = 0;
}

void StateMachine::on_falling_edge_detected(uint32_t now) {
  if (current_state_ == ReadState::F_REQ_WAIT_F_EDGE) {
    time_frame_prefix_started_ = now;
    current_state_ = ReadState::F_REQ_WAIT_R_EDGE;
  }
}

bool StateMachine::on_rising_edge_detected(uint32_t now) {
  if (current_state_ == ReadState::F_REQ_WAIT_R_EDGE) {
    uint32_t duration = now - time_frame_prefix_started_;
    // Check for ~30ms low period
    if (duration > 29000 && duration < 31000) {
      return true; // Indicates a valid start
    } else {
      current_state_ = ReadState::F_REQ_IDLE;
    }
  }
  return false;
}

}  // namespace diesel_heater
}  // namespace esphome
