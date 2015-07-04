/* 
 * The following userspace module uses Kernel user-space
 * interrupts in order to have a working infrastructure
 * for quadrature encoders.
 *
 * References:
 * http://stackoverflow.com/questions/19257624/interrupt-handling-and-user-space-notification
 *
 */

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include "quadrature-encoder.h"

QuadratureEncoder::QuadratureEncoder(void)
{
    int error;
    char cmd[];
    
    /* Set up the GPIO driver direction */
    echo in > /sys/class/gpio/gpio23/direction

    /* Both rising and falling edge support for user-mode interrupts */
    error = system("echo " + "0" + " > /sys/class/gpio/export");
    
    if(error) {
        std::cout << "ERROR: Unable to register GPIO pins" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        std::cout << "INFO: Registered a new quadrature encoder driver (pin";
        std::cout << channel_a;
        std::cout << " : pin";
        std::cout << channel_b;
        std::cout << ")" << std::endl;
    }
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
