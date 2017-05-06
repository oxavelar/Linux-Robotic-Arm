/* 
 * The following userspace module uses Kernel user-space
 * interrupts in order to have a working infrastructure
 * for quadrature encoders.
 *
 * Note: In order to have a cleaner signal tt is better
 * to go and configurue the GPIO pins to have either 10k
 * or 20k pulldowns for less noise at the input.
 *
 * References:
 * https://github.com/tweej/HighLatencyGPIO
 *
 */

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <iomanip>
#include "QuadratureEncoder.h"



QuadratureEncoder::QuadratureEncoder(const int &pin_a, const int &pin_b, const int &rate):
    _encoder_rate(rate)
{
    /* Encoder rate on single edge is 2x, and 4x for both edges */
    GPIO::Edge interrupt_mode;
    if      (_encoder_rate == 2)    interrupt_mode = GPIO::Edge::RISING;
    else if (_encoder_rate == 4)    interrupt_mode = GPIO::Edge::BOTH;
    else    throw std::runtime_error("Invalid encoder rate selected, only 2x or 4x supported");

#if DEBUG
    /* Zero out our debug counters in case of optimizations */
     _channel_a_isr_count = 0;
     _channel_b_isr_count = 0;
     _gpio_processing_error_count = 0;
#endif

    /* Register our local GPIO callbacks to use for SW interrupts */
    _channel_a_callback = std::bind(&QuadratureEncoder::ISR_ChannelA, this);
    _channel_b_callback = std::bind(&QuadratureEncoder::ISR_ChannelB, this);
    
    /* Initialize channels GPIO objects and assign local callbacks */
    _gpio_a = std::unique_ptr<GPIO>(new GPIO(pin_a, interrupt_mode, _channel_a_callback));
    _gpio_b = std::unique_ptr<GPIO>(new GPIO(pin_b, interrupt_mode, _channel_b_callback));
    
    /* Useful information to be printed regarding set-up */
    std::cout << "I: Userspace quadrature encoder created @ (pinA="
              << pin_a
              << " pinB="
              << pin_b << ")" << std::endl; 
    std::cout << "   operating at a rate of " << rate << "x" << std::endl;
}


QuadratureEncoder::~QuadratureEncoder(void)
{
#if DEBUG
    PrintDebugStats();
#endif
}


void QuadratureEncoder::SetParameters(const int &segments)
{
    _segments_per_revolution = segments;
}


double QuadratureEncoder::GetAngle(void)
{
    double degrees;
    degrees = 360.0 * _counter / (double)_segments_per_revolution;
    return degrees;
}


std::chrono::nanoseconds QuadratureEncoder::GetPeriod(void)
{
    return _pulse_period_ns;
}


void QuadratureEncoder::SetZero(void)
{
    _counter = 0;
}


QuadratureEncoder::Direction QuadratureEncoder::GetDirection(void)
{
    return _direction;
}


void QuadratureEncoder::ISR_ChannelA(void)
{
    GPIO_DataProcess();
#ifdef DEBUG
    _channel_a_isr_count++;
#endif
}


void QuadratureEncoder::ISR_ChannelB(void)
{
    GPIO_DataProcess();
#ifdef DEBUG
    _channel_b_isr_count++;
#endif
}


inline void QuadratureEncoder::GPIO_DataProcess(void)
{
    char a, b;
    char current_packed_read;

    /* Convert enum class to actual zero or one */
    _gpio_a->getValue() == GPIO::Value::HIGH ? a = 1 : a = 0;
    _gpio_b->getValue() == GPIO::Value::HIGH ? b = 1 : b = 0;

    /* Convert binary input to decimal value */
    current_packed_read = (b << 1) | (a << 0);
    /* Increment, or decrement depending on matrix */
    auto index = _prev_packed_read * 4 + current_packed_read;
    auto delta = _qem[index % 16];

    /* Put a code guard on illegal encoder train pulse values */
    if (delta == 'x') {
#ifdef DEBUG
        _gpio_processing_error_count++;
        //std::cout << "W: Execution might be too slow, reading wrong values from the encoder" << std::endl;
#endif
        delta = 0;
    }
    
    /* Update our rotation direction now, casting to enum */
    if (delta) _direction = (Direction)delta;
    
    /* Update our local tracking variable */
    _counter += delta;
    
    /* Update our previous reading */
    _prev_packed_read = current_packed_read;
}


void QuadratureEncoder::TrackChannelPulseWidth(void)
{
    /* 
     * Will only be called from a single ISR, a single channel
     * shall provide us with the information for pulse-width
     * tracking. 
     */
    const auto now = std::chrono::high_resolution_clock::now();
    
    if(_internal_toggle++ % 2) {
        _isr_timestamp = now;
    } else {
        /* Calculate the pulsewidth on even transitions */
        auto delta = now - _isr_timestamp;
        _pulse_period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(delta);
    }
}

#ifdef DEBUG

/* The following piece of code is only useful for debug statistics it will collect
 *  data and information regardin the way the interrupts  are collected by the
 *  QuadratureEncoder module, they can be called wherever u want, but right now 
 *  it is only shown by the destructor.
 */

void QuadratureEncoder::PrintDebugStats(void)
{
    double gpio_error_rate = 100 * _gpio_processing_error_count / (double)
                             (_channel_a_isr_count + _channel_b_isr_count);
    std::cout << "D: Internal counter value   " << _counter << std::endl;
    std::cout << "D: ChannelA interrupts      " << _channel_a_isr_count << std::endl;
    std::cout << "D: ChannelB interrupts      " << _channel_b_isr_count << std::endl;
    std::cout << "D: GPIO processing errors   " << _gpio_processing_error_count << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "D: GPIO error rate          " << gpio_error_rate << "%" << std::endl;
    std::cout << std::endl;
}
#endif

