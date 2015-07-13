/* 
 * The following userspace module uses Kernel user-space
 * interrupts in order to have a working infrastructure
 * for quadrature encoders.
 *
 * References:
 * https://github.com/tweej/HighLatencyGPIO
 *
 */

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <chrono>
#include "QuadratureEncoder.h"


QuadratureEncoder::QuadratureEncoder(const uint16_t &pin_a, const uint16_t &pin_b)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Register our local GPIO callbacks to use for SW interrupts */
    _channel_a = std::bind(&QuadratureEncoder::ISR_ChannelA, this);
    _channel_b = std::bind(&QuadratureEncoder::ISR_ChannelB, this);
    
    /* Initialize channel A and channel B counters */

    _gpio_a = new GPIO(pin_a, GPIO::Edge::BOTH, _channel_a);
    _gpio_b = new GPIO(pin_b, GPIO::Edge::BOTH, _channel_b);

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace quadrature encoder initialized @ (pinA=";
    std::cout << pin_a;
    std::cout << " pinB=";
    std::cout << pin_b;
    std::cout << ")" << std::endl;
}


QuadratureEncoder::~QuadratureEncoder(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    delete _gpio_a;
    delete _gpio_b;
    //delete _gpio_z;
}


void QuadratureEncoder::Start(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;    
}


void QuadratureEncoder::Stop(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;    
}


/* Internal Quadrature Encoder ISR Handlers */
void QuadratureEncoder::ISR_ChannelA(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    GPIO::Value pin_value = _gpio_a->getValue();
    fprintf(stdout, "Pin value %d\n\n", pin_value);
}


void QuadratureEncoder::ISR_ChannelB(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    GPIO::Value pin_value = _gpio_b->getValue();
    fprintf(stdout, "Pin value %d\n\n", pin_value);
}


/* External API for the class */
uint32_t QuadratureEncoder::GetPosition(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    uint32_t read_val = _counter_val;
    return read_val;

}


uint32_t QuadratureEncoder::GetPeriod(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    uint32_t read_val = _counter_val;
    return read_val;
}

void QuadratureEncoder::ResetPosition(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    _counter_val = 0;
}


bool QuadratureEncoder::GetDirection(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    return true;
}

