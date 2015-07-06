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
#include "HighLatencyGPIO/GPIO.hh"
#include "quadrature-encoder.h"


QuadratureEncoder::QuadratureEncoder(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Register our local GPIO callbacks to use for SW interrupts */
    _channel_a = std::bind(&ISR_ChannelA);
    _channel_b = std::bind(&ISR_ChannelB);
    _channel_z = std::bind(&ISR_ChannelZ);
    
    /* Initialize channel A and channel B counters */
    _gpio_a = new GPIO(CONFIG_GPIO_PIN_A, GPIO::Edge::BOTH, _channel_a);
    _gpio_b = new GPIO(CONFIG_GPIO_PIN_B, GPIO::Edge::BOTH, _channel_b);
    _gpio_z = new GPIO(CONFIG_GPIO_PIN_Z, GPIO::Edge::BOTH, _channel_z);

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace quadrature encoder initialized @ (pinA=";
    std::cout << CONFIG_GPIO_PIN_A;
    std::cout << " pinB=";
    std::cout << CONFIG_GPIO_PIN_B;
    std::cout << " pinZ=";
    std::cout << CONFIG_GPIO_PIN_Z;
    std::cout << ")" << std::endl;
}

QuadratureEncoder::~QuadratureEncoder(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    delete _gpio_a;
    delete _gpio_b;
    delete _gpio_z;
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
}

void QuadratureEncoder::ISR_ChannelB(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void QuadratureEncoder::ISR_ChannelZ(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}

/* External API for the class */
int32_t QuadratureEncoder::GetPosition(void)
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
