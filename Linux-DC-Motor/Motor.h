#pragma once
#include <chrono>
#include "../HighLatencyPWM/PWM.hh"
#include "../HighLatencyGPIO/GPIO.hh"

#ifndef BASE_PWM_FREQUENCY_HZ
#define BASE_PWM_FREQUENCY_HZ 25000
#endif

#ifndef BASE_PWM_DUTYCYCLE
#define BASE_PWM_DUTYCYCLE 0
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
        void ApplyRangeLimits(const double &percent_l = 0, 
                              const double &percent_h = 100);

        Direction GetDirection(void);
        void SetDirection(const Direction &dir);

        State GetState(void);

    private:
        /* External world interactions to the H-Bridge */
        PWM *_pwm_a, *_pwm_b, *_pwm_active;
        /* Used to keep track of stopped motor */
        double _speed_backup;
        /* We can set hard limits to the percentage of PWM channels */
        double _range_compression_factor;
        double _minimum_percent, _maximum_percent;
        double _minimum_duty, _maximum_duty;
};

