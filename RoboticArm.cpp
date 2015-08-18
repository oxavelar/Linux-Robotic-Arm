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
        _control_stopped = true;
        _AutomaticControlThread.join();
    }
    
    delete Position;
    delete Movement;
}


void RoboticJoint::Init(void)
{
    /* Set the motors running, so the control loop can work on it */
    Movement->Start();
    /* Register our control thread */
    _AutomaticControlThread = std::thread(&RoboticJoint::_AngularControl, this);
    std::cout << "INFO: Joint ID " << _id << " is in our home position" << std::endl;
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
    std::cout << "INFO: Joint ID " << _id << " angular control is now active" << std::endl;

    while(!_control_stopped) {
        
        /* Set angle consists of the interaction between position & movement */
        const double k = 0.30;
        const double actual_angle = Position->GetAngle();
        const double error_angle = _reference_angle - actual_angle;
        
        /* Sign dictates the direction of movement */
        if (error_angle >= 0)    Movement->SetDirection(Motor::Direction::CW);
        else                     Movement->SetDirection(Motor::Direction::CCW);

        std::cout << "k=" << k << std::endl;
        std::cout << "actual=" << actual_angle << std::endl;
        std::cout << "reference=" << _reference_angle << std::endl;
        std::cout << "error=" << error_angle << std::endl;
        std::cout << std::endl;
    
        /* Store the computed proportional value to the movement function */
        Movement->SetSpeed( k * std::abs(error_angle) );
        
        /* Send this task to a low priority state for efficient multi-threading */
        sched_yield();
        
    }

    std::cout << "INFO: Joint ID " << _id << " angular control is now deactivated" << std::endl;
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
         * in order to see if the values difference is less than it
         */
        double difference, init_speed = 20.00, epsilon = 0.001;
        joints[id]->Movement->SetDirection(Motor::Direction::CW);
        joints[id]->Movement->SetSpeed(init_speed);
        joints[id]->Movement->Start();
        do {
            double old = joints[id]->Position->GetAngle();
            usleep(100);
            difference = std::abs(joints[id]->Position->GetAngle() - old);
        } while (difference > epsilon);
        /* Reset the position coordinates, this is our new home position */
        joints[id]->Movement->Stop();
        joints[id]->SetZero();
        
        /* PHASE II: */
        /* Let the the joint correction control thread run and motors start-up */
        joints[id]->Init();

        std::cout << "INFO: joint ID " << id << " was fully initialized" << std::endl;

    }
    
     std::cout << std::endl << "INFO: Success, RoboticArm is now ready to operate!" << std::endl;
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
        theta.push_back( joints[id]->GetAngle() / 180 * M_PI );
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

    /* Temporary working matrix per joint elements */
    //std::array<double, config::joints_nr> theta;
    double theta[config::joints_nr];

    /* Length of the links in meters, read only */
    const auto *L = &config::link_lengths[0];

    switch(_joints_nr)
    {
    case 1:
        theta[0] = 0.0000;
    case 2:
        theta[1] = atan( sqrt( 1 - (pos.x*pos.x + pos.y*pos.y - L[0]*L[0] - L[1]*L[1] / (2 * L[0] * L[1])) ) );
        theta[0] = atan(pos.y / pos.x) - atan( (L[1] * sin(theta[1])) / (L[0] + L[1] * cos(theta[1])) );
        break;
    default:
        /* oxavelar: To extend this to 3 dimensions for N joints */
        std::cout << "ERROR: Unable to calculate for more than 2 joints for now..." << std::cout;
        exit(-127);
    }
    
    /* Update each of the joints their new reference angle */
    for(auto id = 0; id < _joints_nr; id++) {
        joints[id]->SetAngle( theta[id] * 180 / M_PI );
    }
}

