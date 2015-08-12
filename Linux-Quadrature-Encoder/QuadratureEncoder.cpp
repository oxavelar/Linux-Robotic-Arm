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
#include <stdexcept>
#include <cmath>
#include "QuadratureEncoder.h"


QuadratureEncoder::QuadratureEncoder(const int &pin_a, const int &pin_b, const int &rate):
    _encoder_rate(rate)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    /* Encoder rate on single edge is 2x, and 4x for both edges */
    GPIO::Edge interrupt_mode;
    if      (_encoder_rate == 2)    interrupt_mode = GPIO::Edge::RISING;
    else if (_encoder_rate == 4)    interrupt_mode = GPIO::Edge::BOTH;
    else    throw std::runtime_error("Invalid encoder rate selected, only 2x or 4x supported");

    /* Register our local GPIO callbacks to use for SW interrupts */
    _channel_a_callback = std::bind(&QuadratureEncoder::_ISR_ChannelA, this);
    _channel_b_callback = std::bind(&QuadratureEncoder::_ISR_ChannelB, this);
    
    /* Initialize channels GPIO objects and assign local callbacks */
    _gpio_a = new GPIO(pin_a, interrupt_mode, _channel_a_callback);
    _gpio_b = new GPIO(pin_b, interrupt_mode, _channel_b_callback);
    
    /* Useful information to be printed regarding set-up */
    std::cout << "INFO: Userspace quadrature encoder created @ (pinA="
              << pin_a
              << " pinB="
              << pin_b << ")" << std::endl; 
    std::cout << "      operating at a rate of " << rate << "x" << std::endl;
}


QuadratureEncoder::~QuadratureEncoder(void)
{
    delete _gpio_a;
    delete _gpio_b;
}


/// External API for the user, exposed to be used by higher classes
void QuadratureEncoder::SetParameters(const int &segments)
{
    _segments_per_revolution = segments;
}


double QuadratureEncoder::GetAngle(void)
{
    double degrees;
    degrees = 360 * _counter / (double)_segments_per_revolution;
    return degrees;
}


std::chrono::nanoseconds QuadratureEncoder::GetPeriod(void)
{
    std::cout << "INFO: Last pulsewidth duration " << _pulse_period_ns.count() 
              << " ns" << std::endl;

    return _pulse_period_ns;
}


void QuadratureEncoder::SetZero(void)
{
    _counter = 0;
}


QuadratureEncoder::Direction QuadratureEncoder::GetDirection(void)
{
    if(_direction == Direction::CW) {
        std::cout << "INFO: Current direction        +" << std::endl;
    } else
        std::cout << "INFO: Current direction        -" << std::endl;
    return _direction;
}


void QuadratureEncoder::PrintStats(void)
{
    GetDirection();
    std::cout << "INFO: Internal counter value   " << _counter << std::endl;
#ifdef DEBUG
    std::cout << "INFO: ChannelA interrupts      " << _channel_a_isr_cnt << std::endl;
    std::cout << "INFO: ChannelB interrupts      " << _channel_b_isr_cnt << std::endl;
    std::cout << "INFO: Channels history         " << (char*)_channels_history << std::endl;
#endif
}


/// Internal Quadrature Encoder ISR Handlers and methods
void QuadratureEncoder::_ISR_ChannelA(void)
{
    /* Obtain the pulsewidth between transitions, rely only on channel A */
    //_TrackChannelPulseWidth();
    _GPIO_DataProcess();
#ifdef DEBUG
    _TraceHistory();
    _channel_a_isr_cnt++;
#endif
}


void QuadratureEncoder::_ISR_ChannelB(void)
{
    _GPIO_DataProcess();
#ifdef DEBUG
    _TraceHistory();
    _channel_b_isr_cnt++;
#endif
}


void QuadratureEncoder::_GPIO_DataProcess(void)
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
        std::cout << "WARNING: Execution might be too slow, reading wrong values from the encoder" << std::endl;
        delta = 0;
    }
    
    /* Update our rotation direction now, casting to enum */
    if (delta) _direction = (Direction)delta;
    
    /* Update our local tracking variable */
    _counter += delta;
    
    /* Update our previous reading */
    _prev_packed_read = current_packed_read;
}


void QuadratureEncoder::_TrackChannelPulseWidth(void)
{
    /* 
     * Will only be called from a single ISR, a single channel
     * shall provide us with the information for pulse-width
     * tracking. 
     */
    static std::atomic_char toggle;
    const auto now = std::chrono::high_resolution_clock::now();
    
    if(toggle++ % 2) {
        _isr_timestamp = now;
    } else {
        /* Calculate the pulsewidth on even transitions */
        auto delta = now - _isr_timestamp;
        _pulse_period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(delta);
    }
}


void QuadratureEncoder::_TraceHistory(void)
{
    /*
     * Once this is called it is used to store a fake trace
     * of what the packed bus is like for debug purposes.
     */
    static std::atomic_char trace_index;
    _channels_history[trace_index++ % 50] =  _prev_packed_read + '0';
}

