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
#if DEBUG
    _PrintStats();
#endif
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
    return _direction;
}


/// Internal Quadrature Encoder ISR Handlers and methods
void QuadratureEncoder::_ISR_ChannelA(void)
{
    /* Obtain the pulsewidth between transitions, rely only on channel A */
    //_TrackChannelPulseWidth();
    _GPIO_DataProcess();
#ifdef DEBUG
    _FillTraceHistory();
    _channel_a_isr_cnt++;
#endif
}


void QuadratureEncoder::_ISR_ChannelB(void)
{
    _GPIO_DataProcess();
#ifdef DEBUG
    _FillTraceHistory();
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

#define QE_MAX_TRACE_DEPTH 64

void QuadratureEncoder::_PrintStats(void)
{
    std::cout << "INFO: Internal counter value   " << _counter << std::endl;
    std::cout << "INFO: ChannelA interrupts      " << _channel_a_isr_cnt << std::endl;
    std::cout << "INFO: ChannelB interrupts      " << _channel_b_isr_cnt << std::endl;

    /* Trace table is printed */
    std::string trace_header;
    trace_header.resize(QE_MAX_TRACE_DEPTH, ' ');
    trace_header.at(_trace_index) = 'v';
    std::cout << "                               " << trace_header << std::endl;
    std::cout << "INFO: ChannelA history         " << _channel_a_history << std::endl;
    std::cout << "INFO: ChannelB history         " << _channel_b_history << std::endl;
}


void QuadratureEncoder::_FillTraceHistory(void)
{
    /*
     * Once this is called it is used to store a representative trace
     * in a char string what the bus is like for debug purposes.
     */
    _channel_a_history.at(_trace_index) = (_prev_packed_read  &  1) + '0';
    _channel_b_history.at(_trace_index) = (_prev_packed_read  >> 1) + '0';

    /* Positions the character in the buffer, each two characters */
    _trace_index += 2;
    _trace_index = _trace_index % (QE_MAX_TRACE_DEPTH / 2);
}
#endif
