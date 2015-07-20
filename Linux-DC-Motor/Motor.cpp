/* 
 * The following userspace module uses Kernel user-space
 * PWM controls order to have a working infrastructure
 * for DC motors.
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
    _pwm_b = new PWM(pin_pwm_a);
    
    /* Operational values being calculated at default */
    const float t = 1 / (float)BASE_PWM_FREQUENCY_HZ;
    const PWM::Period _pwm_period_ns = t * 1E9;
    const PWM::Duty _pwm_dutycycle_ns = BASE_PWM_DUTYCYCLE * _pwm_period_ns / 100;
    
    /* Set both PWM channels to default values but keep them off */
    _pwm_a->setPeriod(_pwm_period_ns);
    _pwm_a->setDuty(_pwm_dutycycle_ns);
    _pwm_b->setPeriod(_pwm_period_ns);
    _pwm_b->setDuty(_pwm_dutycycle_ns);

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace motor created @ (pinPWM=" 
              << pin_pwm_a
              << " pinB="
              << pin_pwm_b
              << ")" << std::endl;
    std::cout << "      operating on a PWM frequency of "
              << BASE_PWM_FREQUENCY_HZ << "Hz with " << BASE_PWM_DUTYCYCLE
              << "% duty cycle" << std::endl;
}


Motor::~Motor(void)
{
    /* Deleting the PWM object will disable it's output too */
    delete _pwm_a;
    delete _pwm_b;
}


void Motor::Start(void)
{
    _pwm_sel->setState(PWM::State::ENABLED);
}


void Motor::Stop(void)
{
    _pwm_sel->setState(PWM::State::DISABLED);
}


void Motor::SetSpeed(const float &percent)
{
    /* Translates the speed percentage to a PWM duty cycle */
    if(percent <= 100) {
        _pwm_sel->setDuty(_pwm_sel->getDuty() * percent / 100);
    } else {
        throw std::runtime_error("Invalid speed value");
    }
}


float Motor::GetSpeed(void)
{
    float speed;
    /* Reverse translates the PWM duty cycle to speed % */
    speed = float(100 * _pwm_sel->getDuty() / _pwm_sel->getPeriod());
    return(speed);
}

void Motor::SetDirection(const Direction &dir)
{
    /* Obtain the other direction settings, before switching */
    const float speed = GetSpeed();
    const int stopped = IsStopped();
    
    /* Stop the rotation going in that direction */
    if (!stopped) Stop();

    /* Update the active pwm pointer, depending which way you want to go */
    if (dir == Direction::CW)           _pwm_sel = _pwm_b;
    else if (dir == Direction::CCW)     _pwm_sel = _pwm_a;
    
    /* Copying values over to the other rotation direction */
    SetSpeed(speed);
    if (!stopped) Start();
}


int Motor::IsStopped(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    int stopped = 0;
    if ( _pwm_a->getState() == PWM::State::DISABLED ) stopped |= 1;
    if ( _pwm_b->getState() == PWM::State::DISABLED ) stopped |= 1;

    return(stopped);
}

