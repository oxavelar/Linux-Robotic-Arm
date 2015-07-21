#pragma once
#include <iostream>
#include <chrono>
#include <stdint.h>
#include "../HighLatencyPWM/PWM.hh"
#include "../HighLatencyGPIO/GPIO.hh"

#ifndef BASE_PWM_FREQUENCY_HZ
#define BASE_PWM_FREQUENCY_HZ 2000
#endif

#ifndef BASE_PWM_DUTYCYCLE
#define BASE_PWM_DUTYCYCLE 50
#endif


class Motor
{
    public:
        enum class State : char { STOPPED, RUNNING };
        enum class Direction { CCW, CW };

        explicit Motor(const int &pin_pwm, const int &pin_dir);
        virtual ~Motor(void);

        void Stop(void);
        void Start(void);
        float GetSpeed(void);
        void SetSpeed(const float &percent);

        Direction GetDirection(void);
        void SetDirection(const Direction &dir);

        State GetState(void);

    private:
        /* External world interactions to the H-Bridge */
        PWM *_pwm;
        GPIO * _gpio;
};

