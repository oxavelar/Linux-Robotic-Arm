/* 
 * The following class makes use of lower position objects in order
 * to model a different N joints robotic arm.
 *
 */

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include "RoboticArm.h"



int quad_enc_pins[2][2] = {{24, 25}, {27, 26}};
int dc_motor_pins[2] = {3, 5};
/*
    The following is Intel's Galileo layout for the pins.

    +==========+=========+===================+
    |  SYS_FS  |   LABEL |       DESCRIPTION |
    +==========+=========+===================+
    |      24  |     IO6 |   QE Channel A #1 |
    |      25  |    IO11 |   QE Channel B #1 |
    |      26  |     IO8 |   QE Channel A #2 |
    |      27  |     IO7 |   QE Channel B #2 |
    |       3  |    PWM3 |  Motor DC Ctrl #1 |
    |       5  |    PWM5 |  Motor DC Ctrl #2 |
    +==========+=========+================== +

    Note: Galileo's cannot go slower than ~125 Hz on Linux SYSFS PWM.
*/


RoboticArm::RoboticArm(const uint16_t &joints_nr): _joints_nr(joints_nr)
{
    /* Initialize each joint objects */
    for(auto j = 0; j < _joints_nr; j++) {
        
        /* Objects for the position detection */
#ifndef VISUAL_ENCODER
        angular_joints.push_back(new QuadratureEncoder(quad_enc_pins[j][0],
                                                       quad_enc_pins[j][1]));
#else
        angular_joints.push_back(new VisualEncoder());
#endif
        angular_rotors.push_back(new Motor(dc_motor_pins[j]));
    }

    std::cout << "INFO: Created a " << _joints_nr << " joints object" << std::endl;
    
}


RoboticArm::~RoboticArm(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Call each of the positioning objects destructors */
    for(auto j = 0; j < _joints_nr; j++) {
        delete angular_rotors[j];
        delete angular_joints[j];
    }
}


void RoboticArm::Init(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    /* Perform the initialization for each of the joints */
    for(auto j = 0; j < _joints_nr; j++) {
        /* Get the rotors to a known position */
        angular_rotors[j]->SetSpeed(50);
        angular_rotors[j]->Start();
        angular_rotors[j]->Stop();
    }
}


void RoboticArm::DemoCircle(void)
{

}


void RoboticArm::UpdatePosition(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    /* Print all of the joints positions relative to themselves for now */
    for(auto j = 0; j < _joints_nr; j++) {

        angular_rotors[j]->Start();
        usleep(10E03);
        angular_rotors[j]->Stop();

        angular_joints[j]->GetPosition();
        angular_joints[j]->GetPeriod();

        std::cout << std::endl;
    }
}

