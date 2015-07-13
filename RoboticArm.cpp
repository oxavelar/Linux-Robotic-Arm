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
    
    /* Initialize a joint position object for every required one */
    for(auto j = 0; j < joints_nr; j++) {

#ifndef VISUAL_ENCODER
        angular_joints.push_back(new QuadratureEncoder(15 + 2 * j, 16 + 2 * j));
#else
        angular_joints.push_back(new VisualEncoder());
#endif

    }

    std::cout << "INFO: Created a " << _joints_nr << " arm object" << std::endl;
    
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
      angular_joints[j]->GetPosition();
    }
}

