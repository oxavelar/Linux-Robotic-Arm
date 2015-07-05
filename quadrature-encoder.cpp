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
    /* Register our local GPIO callbacks to use for SW interrupts */
    _channel_a = std::bind(&ISR_ChannelA);
    _channel_b = std::bind(&ISR_ChannelB);
    _channel_z = std::bind(&ISR_ChannelZ);
    
    /* Initialize channel A and channel B counters */
    channel_a = new GPIO(15, GPIO::Edge::BOTH, _channel_a);
    channel_b = new GPIO(16, GPIO::Edge::BOTH, _channel_b);
    channel_z = new GPIO(16, GPIO::Edge::BOTH, _channel_z);
    


    /*
    std::cout << "INFO: Registered a new quadrature encoder driver (pin";
    std::cout << channel_a;
    std::cout << " : pin";
    std::cout << channel_b;
    std::cout << ")" << std::endl;
    */
}

QuadratureEncoder::~QuadratureEncoder(void)
{
    return;
}

void QuadratureEncoder::Start(void)
{
    
}

void QuadratureEncoder::Stop(void)
{
    
}

/* Internal Quadrature Encoder ISR Handlers */
void QuadratureEncoder::ISR_ChannelA(void)
{

}

void QuadratureEncoder::ISR_ChannelB(void)
{

}

void QuadratureEncoder::ISR_ChannelZ(void)
{

}
