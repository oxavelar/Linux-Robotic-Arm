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

    /* Defaults to channel A as active */
    _pwm_active = _pwm_a;

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace motor created @ (pinPWM_A=" 
              << pin_pwm_a
              << " pinPWM_B="
              << pin_pwm_b
              << ")" << std::endl;
    std::cout << "      operating on a PWM frequency of "
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
    _pwm_active->setState(PWM::State::ENABLED);
}


void Motor::Stop(void)
{
    _pwm_active->setState(PWM::State::DISABLED);
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
    
    /* Saturate to period value to be safe, and add 1 to avoid zero */
    val = std::min((double)_pwm_active->getPeriod(), val + 1);
    
    if((percent <= 100) and (percent >= 0))
        _pwm_active->setDuty(val);
    else
        throw std::runtime_error("Invalid speed value");
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
    double old_speed = GetSpeed();
    State old_status = GetState();
    Stop();

    /* Move the direction pin depending which way you want to go */
    if     ( dir == Direction::CW )  _pwm_active = _pwm_a;
    else if( dir == Direction::CCW ) _pwm_active = _pwm_b;
    
    /* Override new values with the other channel */
    SetSpeed(old_speed);
    if(old_status == State::RUNNING)
        Start();
}


Motor::State Motor::GetState(void)
{
    State status = State::STOPPED;
    if( _pwm_active->getState() == PWM::State::ENABLED ) status = State::RUNNING;
    return(status);
}

