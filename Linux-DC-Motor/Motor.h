#pragma once
#include <chrono>
#include <stdint.h>
#include "../HighLatencyPWM/PWM.hh"
#include "../HighLatencyGPIO/GPIO.hh"

#ifndef BASE_PWM_FREQUENCY_HZ
#define BASE_PWM_FREQUENCY_HZ 50000
#endif

#ifndef BASE_PWM_DUTYCYCLE
#define BASE_PWM_DUTYCYCLE 50
#endif


class Motor
{
    public:
        enum class State : char { STOPPED, RUNNING };
        enum class Direction { CCW, CW };

        explicit Motor(const int &pin_pwm_a, const int &pin_pwm_b);
        virtual ~Motor(void);

        void Stop(void);
        void Start(void);
        double GetSpeed(void);
        void SetSpeed(const double &percent);

        Direction GetDirection(void);
        void SetDirection(const Direction &dir);

        State GetState(void);

    private:
        /* External world interactions to the H-Bridge */
        PWM *_pwm_a, *_pwm_b, *_pwm_active;
        /* Used to keep track of stopped motor */
        double _speed_backup;
};

