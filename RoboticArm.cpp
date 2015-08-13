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
    /* Stop the automatic control loop thread */
    if (_AutomaticControlThread.joinable()) {
        _control_done = true;
        _AutomaticControlThread.join();
    }
    
    delete Position;
    delete Movement;
}


void RoboticJoint::Init(void)
{
    Movement->Start();
    /* Register our control thread */
    _AutomaticControlThread = std::thread(&RoboticJoint::_AngularControl, this);
}


double RoboticJoint::GetAngle(void)
{
    /* Reads the internal variable, this is used by 
     * the control loop thread, so just reading it is
     * enough information of where the joint is it */
    return _reference_angle;
}


void RoboticJoint::SetAngle(const double &theta)
{
    /* Update the internal variable, the control loop
     * will take charge of getting us here eventually */
    _reference_angle = theta;
}


void RoboticJoint::SetZero(void)
{
    /* This will reset the sensors and the internal state
     * as our zero reference point */
    Position->SetZero();
    _reference_angle = 0;
}


void RoboticJoint::_AngularControl(void)
{
    do {

        /* Set angle consists of the interaction between position & movement */
        const auto k = 0.20;
        const auto actual_angle = Position->GetAngle();
        const auto error_angle = _reference_angle - actual_angle;
        
        /* Sign dictates the direction of movement */
        if (error_angle >= 0)    Movement->SetDirection(Motor::Direction::CW);
        else                     Movement->SetDirection(Motor::Direction::CCW);
    
        /* Store the computed proportional value to the movement function */
        Movement->SetSpeed( k * std::abs(error_angle) );

    } while(!_control_done);
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

        /* PHASE I: */
        /* Get the rotors to a known position on a tight controlled loop 
         * due to rounding aritmethic errors, we use an epsilon comparision
         * in order to see if the values delta is less than it
         */
        double still_moving;
        double init_speed = 5.00;
        double epsilon = 0.001;
        joints[id]->Movement->SetDirection(Motor::Direction::CW);
        joints[id]->Movement->Start();
        do {
            still_moving = joints[id]->Position->GetAngle();
            joints[id]->Movement->SetSpeed(init_speed);
        } while (std::abs(joints[id]->Position->GetAngle() - still_moving) < epsilon);
        /* Reset the position coordinates, this is our reference */
        joints[id]->Movement->Stop();
        joints[id]->SetZero();
        std::cout << "INFO: joint nr " << id << " is in our home position" << std::endl;
        
        /* PHASE II: */
        /* Let the the joint correction control thread run and motors start-up */
        joints[id]->Init();
        std::cout << "INFO: joint nr " << id << " is now ready to be utilized" << std::endl;
    }
    
     std::cout << std::endl << "INFO: Success, RoboticArm is now ready to operate!" << std::endl;
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
    const auto *L = &config::link_lengths[0];
    
    /* Fill our N joints angles in our temporary matrix */
    for(auto id = 0; id < _joints_nr; id++) {
        theta.push_back( joints[id]->GetAngle() * M_PI / (double)180.0 );
    }
    
    
    switch(_joints_nr)
    {
    case 1:
        pos.x = L[0] * cos(theta[0]);
        pos.y = L[0] * sin(theta[0]);
        pos.z = 0;
        break;
    case 2:
        pos.x = L[0] * cos(theta[0]) + L[1] * cos(theta[0] + theta[1]);
        pos.y = L[0] * sin(theta[0]) + L[1] * sin(theta[0] + theta[1]);
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
    const auto *L = &config::link_lengths[0];
    

    switch(_joints_nr)
    {
    case 1:
        break;
    case 2:
        double theta0, theta1;
        theta1 = atan( sqrt( 1 - (pos.x*pos.x + pos.y*pos.y - L[0]*L[0] - L[1]*L[1]) / (2 * L[0] * L[1]) ) );
        theta0 =  atan(pos.y / pos.x) - atan( (L[1] * sin(theta1)) / (L[0] + L[1] * cos(theta1)) );
        theta.push_back(theta0);
        theta.push_back(theta1);
        break;
    default:
        /* oxavelar: To extend this to 3 dimensions for N joints */
        break;
    }
    
    /* Update each of the joints their new reference angle */
    for(auto id = 0; id < _joints_nr; id++) {
        joints[id]->SetAngle( 180 * theta.back() / (double)M_PI );
        theta.pop_back();
    }
}

