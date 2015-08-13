/* 
 * The following code abstracts the position detection
 * system and makes use a webcam system in order to obtain
 * the angular position of the robotic arm.
 */

#include <iostream>
#include <stdexcept>
#include "VisualEncoder.h"


VisualEncoder::VisualEncoder(const int &port) : _port(port)
{
    return;
}


~VisualEncoder::VisualEncoder(void)
{

}


double VisualEncoder::GetAngle(void)
{
    double degrees;
    degrees;
    return degrees;
}


void VisualEncoder::SetZero(void)
{

}

