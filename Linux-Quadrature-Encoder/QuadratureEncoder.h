#pragma once
#include <iostream>
#include <chrono>
#include <stdint.h>
#include "../HighLatencyGPIO/GPIO.hh"

class QuadratureEncoder
{
    public:
        enum class Direction { CCW, CW };

        explicit QuadratureEncoder(const uint16_t &pin_a, const uint16_t &pin_b);
        virtual ~QuadratureEncoder(void);

        int32_t GetPosition(void);
        std::chrono::nanoseconds GetPeriod(void);
        void ResetPosition(void);
        Direction GetDirection(void);

    private:
        /* Pulse train inputs objects from the GPIO class */
        GPIO *_gpio_a, *_gpio_b;
        
        /* GPIO Interrupt routine user code */
        void ISR_ChannelA(void);
        void ISR_ChannelB(void);

        /* Callback references to be used by GPIO class */
        std::function<void(GPIO::Value)> _channel_a_callback;
        std::function<void(GPIO::Value)> _channel_b_callback;
        
        /* Quadrature Encoder Matrix for conversion
           http://letsmakerobots.com/content/how-use-quadrature-encoder */
        uint8_t _prev_packed_read;
        int8_t _qem[16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

        /* Internal state variables */
        int32_t _counter;
        std::chrono::nanoseconds _pulse_period_ns;
        Direction _direction;
        
        /* Used to keep track of the actual interrupt pulse-widths */
        void TrackGPIOPulseWidth(const GPIO *gpio, const GPIO::Value &condition);
        std::chrono::high_resolution_clock::time_point _isr_timestamp;
};
