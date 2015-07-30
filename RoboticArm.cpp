/* 
 * The following classes makes use of lower position objects in order
 * to model a different N joints robotic arm.
 *
 * Forward Kinematics :     ANGLES -> CARTESIAN
 * Inverse Kinematics :     CARTESIAN -> ANGLES
 *
 *
 * References:
 * http://cdn.intechopen.com/pdfs/379.pdf
 */

#include <iostream>
#include <sstream>
#include <cmath>
#include <stdlib.h>
#include <unistd.h>
#include "RoboticArm.h"
#include "RoboticArm_Config.h"


RoboticJoint::RoboticJoint(const int &id) : _id(id)
{
#ifndef VISUAL_ENCODER

    Position = new QuadratureEncoder(config::quad_enc_pins[_id][0],
                                     config::quad_enc_pins[_id][1]);
    /* Set the physical parameters for correct degree measurements
     * this is basically the number of segments per revolution   */
    Position->SetParameters(config::quad_enc_segments[_id]);

#else

    Position = new VisualEncoder(config::visual_enc_port[_id]);

#endif
    /* H-Bridge 2 PWM pins motor abstraction */
    Movement = new Motor(config::dc_motor_pins[_id][0],
                         config::dc_motor_pins[_id][1]);
}


RoboticJoint::~RoboticJoint(void)
{
    delete Position;
    delete Movement;
}


RoboticArm::RoboticArm(void) : _joints_nr(config::joints_nr)
{
    /* Initialize each joint objects with unique ID's */
    for(auto id = 0; id < _joints_nr; id++) {
        joints.push_back(new RoboticJoint(id));
    }
    std::cout << "INFO: Created a " << _joints_nr << " joints arm object" << std::endl;
}


RoboticArm::~RoboticArm(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    
    /* Call each of the joints destructors and stop any movement object */
    for(auto id = 0; id < _joints_nr; id++) {
        joints[id]->Movement->Stop();
        delete joints[id];
    }
}


void RoboticArm::Init(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    /* Perform the initialization for each of the joints */
    for(auto id = 0; id < _joints_nr; id++) {

        /* Get the rotors to a known position on a tight controlled loop */
        int still_moving;
        double init_speed = 80.00;
        joints[id]->Movement->SetDirection(Motor::Direction::CW);
        joints[id]->Movement->Start();
        do {
            still_moving = joints[id]->Position->GetPosition();
            joints[id]->Movement->SetSpeed(init_speed);
            usleep(500);
        } while (joints[id]->Position->GetPosition() != still_moving);
        /* Reset the position coordinates, this is our reference */
        joints[id]->Movement->Stop();
        joints[id]->Position->SetZero();

    }
    std::cout << "INFO: Calibrated all of the joints to a known position" << std::endl;
}


void RoboticArm::DebugMessages(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    /* Print all of the joints positions relative to themselves for now */
    for(auto id = 0; id < _joints_nr; id++) {

        //joints[id]->Movement->SetSpeed(10.0);
        //joints[id]->Movement->Start();
        //usleep(500E03);
        //joints[id]->Movement->Stop();
        
        joints[id]->Position->GetPosition();
        joints[id]->Position->GetDegrees();
        joints[id]->Position->PrintStats();

        std::cout << std::endl;
    }
}

Point RoboticArm::GetPosition(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    /* End of the arm position in a 3D space */
    Point position;
    float x, y, z;

    /* Forward kinematics temporary working matrix */
    std::vector<float> theta;

    /* Length of the links in meters, read only */
    const float *alpha = &config::link_lengths[0];
    
    /* Fill our N joints angles in our temporary matrix */
    for(auto id = 0; id < _joints_nr; id++) {
        theta.push_back(joints[id]->Position->GetDegrees());
    }

    /* 2D and 1D forward kinematics hardcoded */
    switch(_joints_nr)
    {
    case 1:
        x = 0;
        y = 0;
        z = 0;
        break;
    case 2:
        x = alpha[0] * cos(theta[0]) + alpha[1] * cos(theta[0] + theta[1]);
        y = alpha[0] * sin(theta[0]) + alpha[1] * sin(theta[0] + theta[1]);
        z = 0;
        break;
    default:
        /* oxavelar: To extend this to 3 dimensions for N joints */
        x = 0;
        y = 0;
        z = 0;
        break;
    }

    /* Fill in our 3D calculations */
    position.push_back(x);
    position.push_back(y);
    position.push_back(z);
   
    /* C++11 will return a copy of the std::vector, we're safe */
    return position; 
}

