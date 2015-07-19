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


Motor::Motor(const uint16_t &pin_pwm)
{
    /* DC motor control is performed with PWM sysfs abstraction */
    _pwm = new PWM(pin_pwm);
    
    /* Operational values at default */
    _pwm_period_ns = ((1 / BASE_PWM_FREQUENCY_HZ) * 1000 * 1000 * 1000);
    _pwm_dutycycle_ns = (BASE_PWM_DUTYCYCLE * _pwm_period_ns / 100);
    
    _pwm->setPeriod(_pwm_period_ns);
    _pwm->setDuty(_pwm_dutycycle_ns);

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace motor created @ (pinPWM=" 
              << pin_pwm << ")" << std::endl;
    std::cout << "      operating on a PWM frequency of "
              << BASE_PWM_FREQUENCY_HZ << "Hz with " << BASE_PWM_DUTYCYCLE
              << "% duty cycle" << std::endl;
}


Motor::~Motor(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    /* Deleting the PWM object will disable it's output too */
    delete _pwm;
}


void Motor::Start(void)
{
    _pwm->setState(PWM::State::ENABLED);
}


void Motor::Stop(void)
{
    _pwm->setState(PWM::State::DISABLED);
}


void Motor::SetSpeed(const float &percent)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    if(percent <= 100) {
        _pwm->setDuty(_pwm_period_ns * percent / 100);
    } else {
        throw std::runtime_error("Invalid speed value");
    }
}

int Motor::IsStopped(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    int stopped = 0;
    if ( _pwm->getState() == PWM::State::ENABLED ) stopped = 1;
    return(stopped);
}

