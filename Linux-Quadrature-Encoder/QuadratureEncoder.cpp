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
#include <chrono>
#include <stdexcept>
#include "QuadratureEncoder.h"


QuadratureEncoder::QuadratureEncoder(const uint16_t &pin_a, const uint16_t &pin_b)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Register our local GPIO callbacks to use for SW interrupts */
    _channel_a_callback = std::bind(&QuadratureEncoder::ISR_ChannelA, this);
    _channel_b_callback = std::bind(&QuadratureEncoder::ISR_ChannelB, this);
    
    /* Initialize channels GPIO objects and assign local callbacks */
    _gpio_a = new GPIO(pin_a, GPIO::Edge::BOTH, _channel_a_callback);
    _gpio_b = new GPIO(pin_b, GPIO::Edge::BOTH, _channel_b_callback);

    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace quadrature encoder created @ (pinA="
              << pin_a
              << " pinB="
              << pin_b
              << ")" << std::endl;
}


QuadratureEncoder::~QuadratureEncoder(void)
{
    delete _gpio_a;
    delete _gpio_b;
}


/* External API for the user, exposed to be used by higher classes */
int32_t QuadratureEncoder::GetPosition(void)
{
    std::cout << "INFO: Current position counter " << _counter << std::endl;
    return _counter;
}


std::chrono::nanoseconds QuadratureEncoder::GetPeriod(void)
{
    std::cout << "INFO: Pulse-width duration " << _pulse_period_ns.count() 
              << " nanoseconds" << std::endl;

    return _pulse_period_ns;
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


/* Internal Quadrature Encoder ISR Handlers */
void QuadratureEncoder::ISR_ChannelA(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    int8_t delta;
    uint8_t a, b;
    uint8_t current_packed_read;

    /* Obtain the pulse-width from last irq up to when the pin becomes LOW */
    TrackGPIOPulseWidth(_gpio_a, GPIO::Value::LOW);

    /* Convert enum class to actual zero or one */
    _gpio_a->getValue() == GPIO::Value::HIGH ? a = 1 : a = 0;
    _gpio_b->getValue() == GPIO::Value::HIGH ? b = 1 : b = 0;

    /* Convert binary input to decimal value */
    current_packed_read = (a << 1) | (b << 0);
    /* Increment, or decrement depending on matrix */
    delta = _qem[_prev_packed_read * 4 + current_packed_read];
    
    /* Update our rotation direction now */
    if(delta == -1)            _direction =  Direction::CCW;
    else if (delta == 1)       _direction =  Direction::CW;
    
    /* Update our local tracking variable */
    _counter += delta;
    
    /* Update our previous reading */
    _prev_packed_read = current_packed_read;
}


void QuadratureEncoder::ISR_ChannelB(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    int8_t delta;
    uint8_t a, b;
    uint8_t current_packed_read;

    /* Convert enum class to actual zero or one */
    _gpio_a->getValue() == GPIO::Value::HIGH ? a = 1 : a = 0;
    _gpio_b->getValue() == GPIO::Value::HIGH ? b = 1 : b = 0;
    
    /* Obtain the pulse-width from last irq up to when the pin becomes LOW */
    TrackGPIOPulseWidth(_gpio_b, GPIO::Value::LOW);

    /* Convert binary input to decimal value */
    current_packed_read = (a << 1) | (b << 0);
    /* Increment, or decrement depending on matrix */
    delta = _qem[_prev_packed_read * 4 + current_packed_read];
    
    /* Update our rotation direction now */
    if(delta == -1)            _direction =  Direction::CCW;
    else if (delta == 1)       _direction =  Direction::CW;
    
    /* Update our local tracking variable */
    _counter += delta;
    
    /* Update our previous reading */
    _prev_packed_read = current_packed_read;
}


void QuadratureEncoder::TrackGPIOPulseWidth(const GPIO *gpio,
                                            const GPIO::Value &condition)
{
    const auto now = std::chrono::high_resolution_clock::now();
    
    if(gpio->getValue() != condition) {
        /* If the GPIO went low obtain the delta keep it stored */
        _isr_timestamp = now;
    } else {
        /* Else, calculate the pulse-width and save it */
        auto delta = now - _isr_timestamp;
        _pulse_period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(delta);
    }
}

