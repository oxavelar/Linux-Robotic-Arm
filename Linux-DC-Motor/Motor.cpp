/* 
 * The following userspace module uses Kernel user-space
 * PWM controls order to have a working infrastructure
 * for DC motors. Using this class for H-Bridge PWM cont-
 * -rolled DC motors.
 *
 * References:
 * https://github.com/oxavelar/HighLatencyPWM
 *
 */

#include <iostream>
#include <stdexcept>
#include "Motor.h"


Motor::Motor(const int &pin_pwm_a, const int &pin_pwm_b)
{
    /* DC motor control is performed with PWM sysfs abstraction */
    _pwm_a = new PWM(pin_pwm_a);
    _pwm_b = new PWM(pin_pwm_b);
    
    /* Operational values being calculated for default base freq */
    const double t = (1 / (double)BASE_PWM_FREQUENCY_HZ);
    const PWM::Period _pwm_period_ns = (t * 1E9);
    const PWM::Duty _pwm_dutycycle_ns = (BASE_PWM_DUTYCYCLE * t * 1E9 / 100);
    
    _pwm_a->setPeriod(_pwm_period_ns);
    _pwm_b->setPeriod(_pwm_period_ns);
    _pwm_a->setDuty(_pwm_dutycycle_ns);
    _pwm_b->setDuty(_pwm_dutycycle_ns);

    /* Starts up the PWM pins */    
    _pwm_a->setState(PWM::State::ENABLED);
    _pwm_b->setState(PWM::State::ENABLED);

    /* Defaults to channel A as active */
    _pwm_active = _pwm_a;

    /* Duty value limits ranges from 0% to 100% */
    ApplyRangeLimits();

    /* Useful information to be printed regarding set-up */
    std::cout << "I: Userspace motor created @ (pinPWM_A=" 
              << pin_pwm_a
              << " pinPWM_B="
              << pin_pwm_b
              << ")" << std::endl;
    std::cout << "   operating on a PWM frequency of "
              << BASE_PWM_FREQUENCY_HZ << "Hz with " << BASE_PWM_DUTYCYCLE
              << "% duty cycle" << std::endl;
}


Motor::~Motor(void)
{
    /* Deleting the PWM object will disable the output */
    delete _pwm_a;
    delete _pwm_b;
}


void Motor::Start(void)
{
    SetSpeed(_speed_backup);
}


void Motor::Stop(void)
{
    _speed_backup = GetSpeed();

    /* Set both PWM outputs to the same lowest value */
    _pwm_a->setDuty(0);
    _pwm_b->setDuty(0);
}


double Motor::GetSpeed(void)
{
    /* Reverse translates the PWM duty cycle to speed % */
    double speed;
    
    speed = _pwm_active->getDuty() - _minimum_duty;
    speed = speed / (double)_range_compression_factor;
    speed = 100 * speed / (double)_maximum_duty;
    
    return(speed);
}


void Motor::SetSpeed(const double &percent)
{
    /* Translates the speed percentage to a PWM duty cycle */
    double val = (_maximum_duty - _minimum_duty) * percent / (double)100;
    
    /* Saturate our value range to fit our conditions */
    val = std::min(val + _minimum_duty, _maximum_duty);
    
    /* Value is now protected from 0 to 100 ranges at most */
    _pwm_active->setDuty(val);
}


void Motor::ApplyRangeLimits(const double &percent_l, const double &percent_h)
{
    if (percent_h > percent_l) {
        /* Used for calculations later */
        _minimum_percent = percent_l;
        _maximum_percent = percent_h;
        
        /* Minimum motor operating duty cycle */
        _minimum_duty = (double)_pwm_active->getPeriod() / (double)100 * percent_l;
        /* Maximum motor operating duty cycle */
        _maximum_duty = (double)_pwm_active->getPeriod() / (double)100 * percent_h;
        
        /* Updates our compressed range for calculations */
        _range_compression_factor = (percent_h - percent_l) / (double)100;
    } else {
        throw std::runtime_error("Invalid speed range limit values");
    }
}


Motor::Direction Motor::GetDirection(void)
{
    Direction dir = Direction::CW;
    if     ( _pwm_active == _pwm_a ) return Direction::CW;
    else if( _pwm_active == _pwm_b ) return Direction::CCW;
    return(dir);
}


void Motor::SetDirection(const Direction &dir)
{
    /* Save speed and state to perform the new setting switch */
    Stop();

    /* Move the direction pin depending which way you want to go */
    if     ( dir == Direction::CW )  _pwm_active = _pwm_a;
    else if( dir == Direction::CCW ) _pwm_active = _pwm_b;
    
    /* Override new values from the other channel */
    Start();
}


Motor::State Motor::GetState(void)
{
    /* Assume it is running for now ...*/
    State status = State::RUNNING;

    /* If both pwm duties are equal, it means it is stopped */
    if( _pwm_a->getDuty() == _pwm_a->getDuty() ) status = State::STOPPED;
    
    return(status);
}

