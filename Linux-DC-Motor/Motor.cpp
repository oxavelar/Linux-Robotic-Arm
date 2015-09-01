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

    /* Duty value limits according to target time values */
    _maximum_duty = 100 * _pwm_period_ns;
    _minimum_duty =   0 * _pwm_period_ns;

    /* Starts up the PWM pins */    
    _pwm_a->setState(PWM::State::ENABLED);
    _pwm_b->setState(PWM::State::ENABLED);

    /* Defaults to channel A as active */
    _pwm_active = _pwm_a;

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
    double speed;
    /* Reverse translates the PWM duty cycle to speed % */
    speed = (double)_pwm_active->getDuty() * 100 / (double)_pwm_active->getPeriod();
    return(speed);
}


void Motor::SetSpeed(const double &percent)
{
    /* Translates the speed percentage to a PWM duty cycle */
    double val = (double)_pwm_active->getPeriod() * percent / (double)100;
    
    /* Saturate our value range to fit our conditions */
    val = std::min(std::max(val, _minimum_duty), _maximum_duty);
    
    if((int)percent >= 0) {
        _pwm_active->setDuty(val);
    } else {
        //throw std::runtime_error("Unsupported negative speed values");
    }
}


void Motor::SetMinSpeed(const double &percent)
{
    /* Minimum motor operating duty cycle */
    _minimum_duty = (double)_pwm_active->getPeriod() * percent / (double)100;
}


void Motor::SetMaxSpeed(const double &percent)
{
    /* Maximum motor operating duty cycle */
    _maximum_duty = (double)_pwm_active->getPeriod() * percent / (double)100;
}

Motor::Direction Motor::GetDirection(void)
{
    Direction dir = Direction::CW;
    if     ( _pwm_active == _pwm_a ) dir = Direction::CW;
    else if( _pwm_active == _pwm_b ) dir = Direction::CCW;
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

