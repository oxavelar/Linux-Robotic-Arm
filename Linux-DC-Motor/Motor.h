#pragma once
#include <iostream>
#include <chrono>
#include <stdint.h>
#include "../HighLatencyPWM/PWM.hh"

#ifndef BASE_PWM_FREQUENCY_HZ
#define BASE_PWM_FREQUENCY_HZ 2000
#endif

#ifndef BASE_PWM_DUTYCYCLE
#define BASE_PWM_DUTYCYCLE 50
#endif

class Motor
{
    public:
        enum class Direction { CCW, CW };

        explicit Motor(const int &pin_pwm_a, const int &pin_pwm_b);
        virtual ~Motor(void);

        void Start(void);
        void Stop(void);
        void SetSpeed(const float &percent);
        float GetSpeed(void);

        void SetDirection(const Direction &dir);

        int IsStopped(void);

    private:
        /* PWM control objects from the class */
        PWM *_pwm_a, *_pwm_b, *_pwm_sel;
};
