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
#include <chrono>
#include "QuadratureEncoder.h"


QuadratureEncoder::QuadratureEncoder(const uint16_t &pin_a, const uint16_t &pin_b)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Register our local GPIO callbacks to use for SW interrupts */
    _channel_a_callback = std::bind(&QuadratureEncoder::ISR_ChannelA, this);
    _channel_b_callback = std::bind(&QuadratureEncoder::ISR_ChannelB, this);
    
    /* Initialize channel A and channel B counters */

    _gpio_a = new GPIO(pin_a, GPIO::Edge::BOTH, _channel_a_callback);
    _gpio_b = new GPIO(pin_b, GPIO::Edge::BOTH, _channel_b_callback);

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
    
    uint8_t a, b;
    uint8_t delta;
    uint8_t current_packed_read;

    /* Convert enum class to actual zero or one */
    _gpio_a->getValue() == GPIO::Value::HIGH ? a = 1 : a = 0;
    _gpio_b->getValue() == GPIO::Value::HIGH ? b = 1 : b = 0;

    /* Convert binary input to decimal value */
    current_packed_read = (a << 1) | (b << 0);
    /* Increment, or decrement depending on matrix */
    delta = _qem[_prev_packed_read * 4 + current_packed_read];
    
    if(delta == -1)
        _direction =  Direction::CCW;
    else if (delta == 1)
        _direction =  Direction::CW;
    
    /* Update our local tracking variable */
    _counter += delta;

    /* Update our previous reading */
    _prev_packed_read = current_packed_read;
}


void QuadratureEncoder::ISR_ChannelB(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    uint8_t a, b;
    uint8_t delta;
    uint8_t current_packed_read;

    /* Convert enum class to actual zero or one */
    _gpio_a->getValue() == GPIO::Value::HIGH ? a = 1 : a = 0;
    _gpio_b->getValue() == GPIO::Value::HIGH ? b = 1 : b = 0;

    /* Convert binary input to decimal value */
    current_packed_read = (a << 1) | (b << 0);
    /* Increment, or decrement depending on matrix */
    delta = _qem[_prev_packed_read * 4 + current_packed_read];
    
    if(delta == -1)
        _direction =  Direction::CCW;
    else if (delta == 1)
        _direction =  Direction::CW;
    
    /* Update our local tracking variable */
    _counter += delta;

    /* Update our previous reading */
    _prev_packed_read = current_packed_read;
}


/* External API for the class */
int32_t QuadratureEncoder::GetPosition(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::cout << "Current local counter " << _counter << std::endl;
    return _counter;
}


uint32_t QuadratureEncoder::GetPeriod(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    return _pulse_period_us;
}

void QuadratureEncoder::ResetPosition(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    _counter = 0;
}


QuadratureEncoder::Direction QuadratureEncoder::GetDirection(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    return _direction;
}

