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

    /* +==========+=========+
       |  SYS_FS  |   LABEL |
       +==========+=========+
       |      24  |     IO6 |
       |      25  |    IO11 |
       |      26  |     IO8 | 
       |      27  |     IO7 |
       +==========+=========+
    */
    int pins[][2] = {{24, 25}, {27, 26}};

    /* Initialize a joint position object for every required one */
    for(auto j = 0; j < _joints_nr; j++) {

#ifndef VISUAL_ENCODER
        angular_joints.push_back(new QuadratureEncoder(pins[j][0], pins[j][1]));
#else
        angular_joints.push_back(new VisualEncoder());
#endif

    }

    std::cout << "INFO: Created a " << _joints_nr << " joints object" << std::endl;
    
}


RoboticArm::~RoboticArm(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Call each of the positioning objects destructors */
    for(auto j = 0; j < _joints_nr; j++) {
      delete angular_joints[j];
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

