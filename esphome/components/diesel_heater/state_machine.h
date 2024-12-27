#pragma once
#include <cinttypes>

namespace esphome {
namespace diesel_heater {

enum class ReadState {
  F_REQ_IDLE,         // Waiting for falling edge then 30ms low
  
  F_REQ_WAIT_F_EDGE,
  F_REQ_WAIT_R_EDGE,  // Rising edge after 30ms low
  F_REQ_READ,         // Reading bits at ~4.04ms intervals
  F_REQ_WAIT_END,
  
  F_RESP_PRE_WAIT,
  F_RESP_WRITE,       // Writing bits at ~4.04ms intervals

  F_RESP_WAIT_F_EDGE,
  F_RESP_WAIT_R_EDGE,
  F_RESP_READ,
  F_RESP_WAIT_END,

  F_REQ_PRE_WAIT,
  F_REQ_WRITE,
  

};

class StateMachine {
 public:
  StateMachine() = default;
  
  void reset(ReadState rs = ReadState::F_REQ_WAIT_F_EDGE);
  
  ReadState current_state() const { return current_state_; }
  void set_state(ReadState state) { current_state_ = state; }

  void on_falling_edge_detected(uint32_t now);
  bool on_rising_edge_detected(uint32_t now);
  
  void increment_bit_index() { current_bit_index_++; }
  uint8_t current_bit_index() const { return current_bit_index_; }
  void set_bit_index(uint8_t idx) { current_bit_index_ = idx; }

  void set_frame_start_time(uint32_t t) { time_frame_prefix_started_ = t; }
  uint32_t frame_start_time() const { return time_frame_prefix_started_; }

  void set_bits_to_read(uint8_t bits) { bits_to_read_ = bits; }
  uint8_t bits_to_read() const { return bits_to_read_; }

 private:
  ReadState current_state_{ReadState::F_REQ_IDLE};
  volatile uint8_t bits_to_read_{0};
  volatile uint8_t current_bit_index_{0};
  volatile uint32_t time_frame_prefix_started_{0};
};

}  // namespace diesel_heater
}  // namespace esphome
