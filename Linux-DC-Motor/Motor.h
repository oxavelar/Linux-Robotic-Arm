#pragma once
#include <iostream>
#include <chrono>
#include <stdint.h>
#include "../HighLatencyPWM/PWM.hh"

#ifndef BASE_PWM_FREQUENCY_HZ
#define BASE_PWM_FREQUENCY_HZ 2000u
#endif

#ifndef BASE_PWM_DUTYCYCLE
#define BASE_PWM_DUTYCYCLE 50u
#endif

class Motor
{
    public:
        enum class Direction { CCW, CW };

        explicit Motor(const uint16_t &pin_pwm);
        virtual ~Motor(void);

        void Start(void);
        void Stop(void);
        void SetSpeed(const float &percent);
        int IsStopped(void);

    private:
        /* PWM control objects from the class */
        PWM *_pwm;

        /* Internal state variables */
        uint32_t _speed;
        
        PWM::Period _pwm_period_ns;
        PWM::Duty _pwm_dutycycle_ns;
};
