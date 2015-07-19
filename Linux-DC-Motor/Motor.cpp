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
#include <unistd.h>
#include "Motor.h"


Motor::Motor(const uint16_t &pin_pwm)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    _pwm = new PWM(pin_pwm);
    
    /* Set some default values for testing */
    auto period = 7000000;
    _pwm->setPeriod(period);
    _pwm->setDuty(0.1 * period);
    _pwm->setState(PWM::State::ENABLED);

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace motor initialized @ (pinPWM=" 
              << pin_pwm << ")" << std::endl;
}


Motor::~Motor(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    _pwm->setState(PWM::State::DISABLED);
    usleep(500000);
    delete _pwm;
}

