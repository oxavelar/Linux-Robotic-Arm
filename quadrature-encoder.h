#pragma once
#include <iostream>
#include <stdint.h>
#include "HighLatencyGPIO/GPIO.hh"

class QuadratureEncoder
{
    public:
        QuadratureEncoder(void);
        ~QuadratureEncoder(void);

        void Init(void);
        void Start(void);
        void Stop(void);

    private:
        /* Pulse train inputs from GPIO class */
        GPIO *_gpio_a, *_gpio_b, *_gpio_z;
        
        /* GPIO Interrupt routine user code */
        static void ISR_ChannelA(void);
        static void ISR_ChannelB(void);
        static void ISR_ChannelZ(void);

        /* Callback references to be used by GPIO class */
        std::function<void(GPIO::Value)> _channel_a;
        std::function<void(GPIO::Value)> _channel_b;
        std::function<void(GPIO::Value)> _channel_z;

        /* Local counter variables */
        uint32_t counter_val;

};
