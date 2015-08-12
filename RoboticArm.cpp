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

    Position = new QuadratureEncoder(config::quad_encoder_pins[_id][0],
                                     config::quad_encoder_pins[_id][1],
                                     config::quad_encoder_rate);
    /* Set the physical parameters for correct degree measurements
     * this is basically the number of segments per revolution   */
    Position->SetParameters(config::quad_encoder_segments[_id]);

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


double RoboticJoint::GetAngle(void)
{
    double angle = Position->GetAngle();
    std::cout << __PRETTY_FUNCTION__ << " = " << angle << std::endl;
    return angle;
}


void RoboticJoint::SetAngle(const double &theta)
{
    /* Set angle consists of the interaction between Position & Movement */
    const auto initial_angle = Position->GetAngle();
    
    if( initial_angle != theta ) {
        std::cout << "INFO: Commanded angle for joint " << _id << " is different";
        std::cout << "      Trying to guess the position now!";

        /* Go right, or go left */

        /* Proceed with caution on a tight controlled loop */
        Movement->SetSpeed(10.0);
        Movement->Start();
        Movement->SetDirection(Motor::Direction::CW);
        std::cout << "INFO: Going 1 second CW!" << std::endl;
        usleep(1E06);
        Movement->SetDirection(Motor::Direction::CCW);
        std::cout << "INFO: Going 1 second CCW!" << std::endl;
        usleep(1E06);
        Movement->Stop();

    }
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
        double still_moving;
        double init_speed = 80.00;
        joints[id]->Movement->SetDirection(Motor::Direction::CW);
        joints[id]->Movement->Start();
        do {
            still_moving = joints[id]->Position->GetAngle();
            joints[id]->Movement->SetSpeed(init_speed);
            usleep(500);
        } while ((int)joints[id]->Position->GetAngle() != (int)still_moving);
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

        joints[id]->Position->PrintStats();

        std::cout << std::endl;
    }
}


void RoboticArm::GetPosition(Point &pos)
{
    /* Makes use of forward kinematics in order to get position */

    /* Temporary working matrix */
    std::vector<double> theta;

    /* Length of the links in meters, read only */
    const auto *alpha = &config::link_lengths[0];
    
    /* Fill our N joints angles in our temporary matrix */
    for(auto id = 0; id < _joints_nr; id++) {
        theta.push_back( joints[id]->GetAngle() );
    }
    
    
    switch(_joints_nr)
    {
    case 1:
        pos.x = 0;
        pos.y = 0;
        pos.z = 0;
        break;
    case 2:
        pos.x = alpha[0] * cos(theta[0]) + alpha[1] * cos(theta[0] + theta[1]);
        pos.y = alpha[0] * sin(theta[0]) + alpha[1] * sin(theta[0] + theta[1]);
        pos.z = 0;
        break;
    default:
        /* oxavelar: To extend this to 3 dimensions for N joints */
        pos.x = 0;
        pos.y = 0;
        pos.z = 0;
        break;
    }
}


void RoboticArm::SetPosition(const Point &pos)
{
    /* Makes use of inverse kinematics in order to set position */

    /* Temporary working matrix */
    std::vector<double> theta;

    /* Length of the links in meters, read only */
    const auto *alpha = &config::link_lengths[0];
    

    switch(_joints_nr)
    {
    case 1:
        break;
    case 2:
        double theta0, theta1;
        theta1 = atan( sqrt( 1 - (pos.x*pos.x + pos.y*pos.y - alpha[0]*alpha[0] - alpha[1]*alpha[1]) / (2 * alpha[0] * alpha[1]) ) );
        theta0 =  atan(pos.y / pos.x) - atan( (alpha[1] * sin(theta1)) / (alpha[0] + alpha[1] * cos(theta1)) );
        theta.push_back(theta0);
        theta.push_back(theta1);
        break;
    default:
        /* oxavelar: To extend this to 3 dimensions for N joints */
        break;
    }
    
    /* Command each of the joints to the desired angle */
    for(auto id = 0; id < _joints_nr; id++) {
        joints[id]->SetAngle( theta.back() );
        theta.pop_back();
    }
}

