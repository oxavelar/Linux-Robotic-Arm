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
        uint32_t GetPeriod(void);
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
        std::function<void(GPIO::Value)> _channel_a_callback;
        std::function<void(GPIO::Value)> _channel_b_callback;
        std::function<void(GPIO::Value)> _channel_z_callback;
        
        /* Quadrature Encoder Matrix for conversion
           http://letsmakerobots.com/content/how-use-quadrature-encoder */
        uint8_t _prev_read;
        int _qem[16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

        /* Internal state variables */
        int32_t _counter;
        uint32_t _pulse_period_us;

};
