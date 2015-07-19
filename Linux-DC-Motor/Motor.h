#pragma once
#include <iostream>
#include <chrono>
#include <stdint.h>
#include "../HighLatencyPWM/PWM.hh"

class Motor
{
    public:
        enum class Direction { CCW, CW };

        explicit Motor(const uint16_t &pin_pwm);
        virtual ~Motor(void);

    private:
        /* PWM control objects from the class */
        PWM *_pwm;

        /* Internal state variables */
        int32_t _speed;
};
