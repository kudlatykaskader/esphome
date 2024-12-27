#include <Arduino.h>
#include "esphome/core/hal.h"

#include "dh_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace diesel_heater {

DHProtocol *DHProtocol::instance_ = nullptr;

void InterruptPlatform::attach_pin_interrupt(InternalGPIOPin *pin, bool rising, void (*callback)(void)) {
  this->detach_pin_interrupt(pin);
  attachInterrupt(digitalPinToInterrupt(pin->get_pin()), callback, rising ? RISING : FALLING);
}

void InterruptPlatform::detach_pin_interrupt(InternalGPIOPin *pin) { detachInterrupt(pin->get_pin()); }

void InterruptPlatform::init_timer() {
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(1000000);
}

void InterruptPlatform::start_timer(uint32_t period, void (*callback)(void)) {
  timer1_attachInterrupt(callback);
  timer1_write(period);
}

void InterruptPlatform::stop_timer() {
  timer1_detachInterrupt();
  timer1_write(0);
}

void DHProtocol::attach_request_received_callback(void (*callback)(uint8_t data)) {
  this->request_received_callback_ = callback;
}

void DHProtocol::attach_response_received_callback(void (*callback)(uint16_t data)) {
  this->response_received_callback_ = callback;
}

void DHProtocol::add_request(uint8_t data) { this->request_queue_.push(data); }

// start the protocol (configure pins, start interrupts)
void DHProtocol::start() {
  if (this->data_pin_ != nullptr) {
    this->data_pin_->setup();
    this->data_pin_->pin_mode(gpio::FLAG_INPUT);
  }
  DHProtocol::instance_ = this;
  platform_->attach_pin_interrupt(this->data_pin_, false, on_pin_isr);
  platform_->init_timer();
  sm_.set_state(ReadState::F_REQ_WAIT_F_EDGE);
}

void DHProtocol::stop() {
  platform_->detach_pin_interrupt(this->data_pin_);
  platform_->stop_timer();
}

void IRAM_ATTR DHProtocol::on_pin_interrupt() {
  uint32_t now = micros();
  bool valid_start;
  switch (sm_.current_state()) {
    case ReadState::F_REQ_WAIT_F_EDGE:
      sm_.set_frame_start_time(now);
      sm_.set_state(ReadState::F_REQ_WAIT_R_EDGE);
      platform_->attach_pin_interrupt(this->data_pin_, true, on_pin_isr);
      break;

    case ReadState::F_REQ_WAIT_R_EDGE: {
      platform_->detach_pin_interrupt(this->data_pin_);
      valid_start = sm_.on_rising_edge_detected(now);
      if (valid_start) {
        platform_->start_timer(TIME_PERIOD_4040us / 2, on_timer_isr);
        sm_.set_state(ReadState::F_REQ_READ);
        sm_.set_bits_to_read(23);
        sm_.set_bit_index(0);
      } else {
        sm_.set_state(ReadState::F_REQ_WAIT_F_EDGE);
        platform_->attach_pin_interrupt(this->data_pin_, false, on_pin_isr);
      }
      break;

      case ReadState::F_RESP_WAIT_F_EDGE:
        platform_->detach_pin_interrupt(this->data_pin_);
        // platform_->stop_timer();
        sm_.set_state(ReadState::F_RESP_WAIT_R_EDGE);
        platform_->attach_pin_interrupt(this->data_pin_, true, on_pin_isr);
        break;

      case ReadState::F_RESP_WAIT_R_EDGE:
        platform_->detach_pin_interrupt(this->data_pin_);
        platform_->start_timer(TIME_PERIOD_4040us / 2, on_timer_isr);
        sm_.set_state(ReadState::F_RESP_READ);
        sm_.set_bits_to_read(47);
        sm_.set_bit_index(0);
        break;

      case ReadState::F_RESP_WAIT_END:
        platform_->detach_pin_interrupt(this->data_pin_);
        if (this->response_received_callback_ != nullptr) {
          this->response_received_callback_(this->decode_response(this->current_response_));
        }

        if (this->request_queue_.size() > 0) {
          uint8_t request = this->request_queue_.front();
          this->request_queue_.pop();
          this->encode_request(request, this->current_request_);
          sm_.set_state(ReadState::F_REQ_PRE_WAIT);
          platform_->start_timer(TIME_PERIOD_1700us, on_timer_isr);
          sm_.set_bits_to_read(23);
          sm_.set_bit_index(0);

        } else {
          sm_.set_state(ReadState::F_REQ_WAIT_F_EDGE);
          platform_->attach_pin_interrupt(this->data_pin_, false, on_pin_isr);
        }
        break;

      case ReadState::F_REQ_WAIT_END:
        sm_.set_state(ReadState::F_RESP_WAIT_F_EDGE);
        platform_->attach_pin_interrupt(this->data_pin_, false, on_pin_isr);
        platform_->start_timer(TIME_PERIOD_4040us, on_timer_isr);

        if (this->request_received_callback_ != nullptr) {
          // ESP_LOGD("", "Request received: 0x%02X", this->decode_request(this->current_request_));
          this->request_received_callback_(this->decode_request(this->current_request_));
        }
        break;

      default:
        break;
    }
  }
}

void IRAM_ATTR DHProtocol::on_timer_interrupt() {
  switch (sm_.current_state()) {
    case ReadState::F_REQ_READ: {
      uint8_t bit = (uint8_t) digitalRead(this->data_pin_->get_pin());
      current_request_[sm_.current_bit_index()] = bit;
      sm_.increment_bit_index();

      if (sm_.current_bit_index() > sm_.bits_to_read()) {
        sm_.set_state(ReadState::F_REQ_WAIT_END);
        platform_->stop_timer();
        platform_->attach_pin_interrupt(this->data_pin_, true, on_pin_isr);
      } else {
        platform_->start_timer(TIME_PERIOD_4040us, on_timer_isr);
      }
      break;
    }

    case ReadState::F_RESP_READ:
      current_response_[sm_.current_bit_index()] = (uint8_t) digitalRead(this->data_pin_->get_pin());
      sm_.increment_bit_index();

      if (sm_.current_bit_index() > sm_.bits_to_read()) {
        sm_.set_state(ReadState::F_RESP_WAIT_END);
        platform_->stop_timer();
        platform_->attach_pin_interrupt(this->data_pin_, true, on_pin_isr);
      } else {
        platform_->start_timer(TIME_PERIOD_4040us, on_timer_isr);
      }
      break;

    case ReadState::F_REQ_PRE_WAIT:
      sm_.set_state(ReadState::F_REQ_WRITE);
      platform_->start_timer(TIME_PERIOD_30ms, on_timer_isr);
      if (this->data_pin_ != nullptr) {
        this->data_pin_->pin_mode(gpio::FLAG_OUTPUT);
        digitalWrite(this->data_pin_->get_pin(), LOW);
      }
      break;

    case ReadState::F_REQ_WRITE:
      platform_->start_timer(TIME_PERIOD_4040us, on_timer_isr);
      digitalWrite(this->data_pin_->get_pin(), this->current_request_[sm_.current_bit_index()]);
      sm_.increment_bit_index();
      if (sm_.current_bit_index() > 24) {
        sm_.set_state(ReadState::F_RESP_WAIT_F_EDGE);
        platform_->stop_timer();
        platform_->attach_pin_interrupt(this->data_pin_, false, on_pin_isr);
        sm_.set_bits_to_read(47);
        sm_.set_bit_index(0);
        if (this->data_pin_ != nullptr)
          this->data_pin_->pin_mode(gpio::FLAG_INPUT);
      }
      break;
    case ReadState::F_RESP_WAIT_F_EDGE: // Timed out on awaiting response
      platform_->stop_timer();
      sm_.reset(ReadState::F_REQ_WAIT_F_EDGE);
      platform_->attach_pin_interrupt(this->data_pin_, false, on_pin_isr);
      break;
    default:
      break;
  }
}

void IRAM_ATTR DHProtocol::on_pin_isr() {
  if (DHProtocol::instance_ != nullptr) {
    DHProtocol::instance_->on_pin_interrupt();
  }
}

void IRAM_ATTR DHProtocol::on_timer_isr() {
  if (DHProtocol::instance_ != nullptr) {
    DHProtocol::instance_->on_timer_interrupt();
  }
}

uint8_t DHProtocol::decode_request(const bool *input24bits) {
  uint8_t result = 0;
  for (int i = 0; i < 8; i++) {
    bool bit = input24bits[i * 3 + 1];  // middle bit is the data bit
    result = (uint8_t) ((result << 1) | (bit ? 1 : 0));
  }
  return result;
}

uint16_t DHProtocol::decode_response(const bool *input48bits) {
  uint16_t result = 0;
  for (int i = 0; i < 16; i++) {
    result |= input48bits[i * 3 + 1] << (15 - i);
  }
  return result;
}

void DHProtocol::encode_response(uint16_t response_data, bool *output48bits) {
  for (int i = 0; i < 16; i++) {
    bool bit = (response_data & (1 << (15 - i))) != 0;
    *output48bits++ = true;   // '1'
    *output48bits++ = bit;    // data bit
    *output48bits++ = false;  // '0'
  }
}

void DHProtocol::encode_request(uint8_t request_data, bool *output24bits) {
  for (int i = 0; i < 8; i++) {
    bool bit = (request_data & (0x80 >> i)) != 0;  // Extract bit i
    // Encode as 1 bit 0
    *output24bits++ = true;   // '1'
    *output24bits++ = bit;    // bit value
    *output24bits++ = false;  // '0'
  }
}

}  // namespace diesel_heater
}  // namespace esphome
