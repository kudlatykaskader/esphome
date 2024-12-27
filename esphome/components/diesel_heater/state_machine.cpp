#include "state_machine.h"
#include <Arduino.h>

namespace esphome {
namespace diesel_heater {

void StateMachine::reset(ReadState rs) {
  current_state_ = rs;
  bits_to_read_ = 0;
  current_bit_index_ = 0;
  time_frame_prefix_started_ = 0;
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
