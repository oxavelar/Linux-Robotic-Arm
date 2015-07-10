#pragma once
#include <iostream>
#include <stdint.h>
#include "../HighLatencyGPIO/GPIO.hh"

class QuadratureEncoder
{
    public:
        QuadratureEncoder(const uint16_t &pin_a, const uint16_t &pin_b);
        virtual ~QuadratureEncoder(void);

        void Init(void);
        void Start(void);
        void Stop(void);

        int32_t GetPosition(void);
        void ResetPosition(void);
        bool GetDirection(void);

    private:
        /* Pulse train inputs from GPIO class */
        GPIO *_gpio_a, *_gpio_b, *_gpio_z;
        
        /* GPIO Interrupt routine user code */
        void ISR_ChannelA(void);
        void ISR_ChannelB(void);
        void ISR_ChannelZ(void);

        /* Callback references to be used by GPIO class */
        std::function<void(GPIO::Value)> _channel_a;
        std::function<void(GPIO::Value)> _channel_b;
        std::function<void(GPIO::Value)> _channel_z;

        /* Local counter variable */
        int32_t _counter_val;

};
