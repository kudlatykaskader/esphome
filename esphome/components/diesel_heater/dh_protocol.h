#pragma once

#include <queue>
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"

#include "state_machine.h"

namespace esphome {
namespace diesel_heater {

class InterruptPlatform {
 public:
    //  configure pin interrupt on given pin, with given mode and callback
    void attach_pin_interrupt(InternalGPIOPin *pin, bool rising, void (*callback)(void));
    
    //  detach pin interrupt
    void detach_pin_interrupt(InternalGPIOPin *pin);

    void init_timer();
    
    // confgure timer interrupt with given period and callback
    void start_timer(uint32_t period, void (*callback)(void));
    
    // stop timer interrupt
    void stop_timer();
};


class DHProtocol {
    public:
        DHProtocol(InterruptPlatform *platform, InternalGPIOPin *pin) : platform_(platform), data_pin_(pin) {}

        void attach_request_received_callback(void (*callback)(uint8_t data));
        void attach_response_received_callback(void (*callback)(uint16_t data));

        void add_request(uint8_t data);

        // start the protocol (configure pins, start interrupts)
        void start();
        void stop();

    private:

        // Timing constants - adjust multipliers as needed
        static constexpr uint32_t TIME_PERIOD_4040us = 4040 * 5;
        static constexpr uint32_t TIME_PERIOD_30ms   = 30000 * 5;
        static constexpr uint32_t TIME_PERIOD_1700us = 1700 * 5;

        InterruptPlatform *platform_;
        StateMachine sm_;
        InternalGPIOPin *data_pin_;

        static DHProtocol *instance_;

        static void IRAM_ATTR on_pin_isr();
        static void IRAM_ATTR on_timer_isr();

        void IRAM_ATTR on_pin_interrupt();
        void IRAM_ATTR on_timer_interrupt();

        void (*request_received_callback_)(uint8_t data);
        void (*response_received_callback_)(uint16_t data);

        bool current_request_[24];
        bool current_response_[48];

        //  queue of requests
        std::queue<uint8_t> request_queue_;

        uint8_t decode_request(const bool *input24bits);
        uint16_t decode_response(const bool *input48bits);

        void encode_response(uint16_t response_data, bool *output48bits);
        void encode_request(uint8_t request_data, bool *output24bits);

};
    
}  // namespace diesel_heater
}  // namespace esphome
