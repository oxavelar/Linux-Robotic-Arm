/* 
 * The following userspace module uses Kernel user-space
 * PWM controls order to have a working infrastructure
 * for DC motors. The arduino Boards uses an H-Bridge so
 * that we can change direction with a single GPIO pin.
 *
 * References:
 * https://github.com/oxavelar/HighLatencyPWM
 *
 */

#include <iostream>
#include <stdexcept>
#include "Motor.h"


Motor::Motor(const int &pin_pwm, const int &pin_dir)
{
    /* DC motor control is performed with PWM sysfs abstraction */
    _pwm = new PWM(pin_pwm);
    _gpio = new GPIO(pin_dir, GPIO::Direction::OUT);
    
    /* Operational values being calculated for default base freq */
    const double t = (1 / (double)BASE_PWM_FREQUENCY_HZ);
    const PWM::Period _pwm_period_ns = (t * 1E9);
    const PWM::Duty _pwm_dutycycle_ns = (BASE_PWM_DUTYCYCLE * t * 1E9 / 100);
    
    _pwm->setPeriod(_pwm_period_ns);
    _pwm->setDuty(_pwm_dutycycle_ns);
    
    /* Defaults to stopped and clockwise rotation */
    Stop();
    SetDirection(Direction::CW);

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace motor created @ (pinPWM=" 
              << pin_pwm
              << " pinDIR="
              << pin_dir
              << ")" << std::endl;
    std::cout << "      operating on a PWM frequency of "
              << BASE_PWM_FREQUENCY_HZ << "Hz with " << BASE_PWM_DUTYCYCLE
              << "% duty cycle" << std::endl;
}


Motor::~Motor(void)
{
    /* Deleting the PWM object will disable it's output too */
    delete _pwm;
    delete _gpio;
}


void Motor::Start(void)
{
    _pwm->setState(PWM::State::ENABLED);
}


void Motor::Stop(void)
{
    _pwm->setState(PWM::State::DISABLED);
}


double Motor::GetSpeed(void)
{
    double speed;
    /* Reverse translates the PWM duty cycle to speed % */
    speed = (double)_pwm->getDuty() * 100 / (double)_pwm->getPeriod();
    return(speed);
}


void Motor::SetSpeed(const double &percent)
{
    /* Translates the speed percentage to a PWM duty cycle */
    double val = (double)_pwm->getPeriod() * percent / (double)100;
    if((percent <= 100) and (percent >= 0))
        _pwm->setDuty(val);
    else
        throw std::runtime_error("Invalid speed value");
}


Motor::Direction Motor::GetDirection(void)
{
    Direction dir = Direction::CW;
    if     ( _gpio->getValue() == GPIO::Value::LOW  ) dir = Direction::CW;
    else if( _gpio->getValue() == GPIO::Value::HIGH ) dir = Direction::CCW;
    return(dir);
}


void Motor::SetDirection(const Direction &dir)
{
    /* Move the direction pin depending which way you want to go */
    if     ( dir == Direction::CW )  _gpio->setValue(GPIO::Value::LOW);
    else if( dir == Direction::CCW ) _gpio->setValue(GPIO::Value::HIGH);
}


Motor::State Motor::GetState(void)
{
    State status = State::STOPPED;
    if( _pwm->getState() == PWM::State::ENABLED ) status = State::RUNNING;
    return(status);
}

