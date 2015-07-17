/* 
 * The following class makes use of lower position objects in order
 * to model a different N joints robotic arm.
 *
 */

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include "RoboticArm.h"


RoboticArm::RoboticArm(const uint16_t &joints_nr): _joints_nr(joints_nr)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

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

    */
    int quad_enc_pins[][2] = {{24, 25}, {27, 26}};
    int dc_motor_pins[] = {3, 5};

    /* Initialize each joint objects */
    for(auto j = 0; j < _joints_nr; j++) {
        
        /* Objects for the position detection */
#ifndef VISUAL_ENCODER
        angular_joints.push_back(new QuadratureEncoder(quad_enc_pins[j][0],
                                                       quad_enc_pins[j][1]));
#else
        angular_joints.push_back(new VisualEncoder());
#endif
        
        /* Object movement childs */
        angular_rotors.push_back(new PWM(dc_motor_pins[j]));
        /* Initialize the rotor control arguments */
        angular_rotors[j]->setPeriod(5000000);
        angular_rotors[j]->setDuty(2500000);
        angular_rotors[j]->setState(PWM::State::ENABLED);
    }

    std::cout << "INFO: Created a " << _joints_nr << " joints object" << std::endl;
    
}


RoboticArm::~RoboticArm(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Call each of the positioning objects destructors */
    for(auto j = 0; j < _joints_nr; j++) {
      delete angular_joints[j];
      delete angular_rotors[j];
    }
}


void RoboticArm::UpdatePosition(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    /* Print all of the joints positions relative to themselves for now */
    for(auto j = 0; j < _joints_nr; j++) {
        (void) angular_joints[j]->GetPosition();
        (void) angular_joints[j]->GetPeriod();
        std::cout << std::endl;
    }
}

