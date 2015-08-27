/* 
 * The following classes makes use of position objects and movement in order
 * to model a different type of N joints robotic arm. Different kinematics
 * configuration will use different sensors and actuators and solutions.
 *
 * The demo and application models a RR arm on a Intel Galileo2 system running
 * Linux.
 *
 * It makes use of inverse and forward kinematics calculations on different
 * position sensors in order to obtain the rotor angular state, and then spawns
 * unique control threads per joint in order to keep the joints to the reference
 * /target angle value.
 *
 *
 *
 * References:
 * http://cdn.intechopen.com/pdfs/379.pdf
 * http://www.cis.upenn.edu/~badler/gmod/0528a.pdf
 *
 * Authors: Omar X. Avelar & Juan C. Razo
 *
 */

#include <iostream>
#include <sstream>
#include <cmath>
#include <atomic>
#include <stdlib.h>
#include <unistd.h>
#include "toolbox.h"
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
    if (AutomaticControlThread.joinable()) {
        _control_thread_stop_event = true;
        AutomaticControlThread.join();
    }
    
    /* Kill off any movement */
    Movement->Stop();
    
    delete Position;
    delete Movement;
}


void RoboticJoint::Init(void)
{
    /* Set the motors running, so the control loop can work on it */
    Movement->Start();
    logger << "INFO: Joint ID " << _id << " is in our home position" << std::endl;
    /* Register our control thread */
    AutomaticControlThread = std::thread(&RoboticJoint::AngularControl, this);
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


void RoboticJoint::AngularControl(void)
{
    logger << "INFO: Joint ID " << _id << " angular control is now active" << std::endl;

    while(!_control_thread_stop_event) {
        
        /* Consists of the interaction between position & movement */
        const auto k = 8;
        const auto actual_angle = Position->GetAngle();
        const auto error_angle = _reference_angle - actual_angle;
        
        /* Sign dictates the direction of movement */
        if (error_angle >= 0)    Movement->SetDirection(Motor::Direction::CCW);
        else                     Movement->SetDirection(Motor::Direction::CW);
        
        /* Store the motor control value to the movement function */
        Movement->SetSpeed( k * std::abs(error_angle) );
        
#ifdef DEBUG
        logger << "actual=" << actual_angle << std::endl;
        logger << "reference=" << _reference_angle << std::endl;
        logger << "error=" << error_angle << std::endl;
        logger << std::endl;
        logger << "speed=" << k * std::abs(error_angle) << "%" << std::endl;
        logger << "speed=" << Movement->GetSpeed() << "%" << std::endl;
        logger << std::endl;
#endif
        
        /* Send this task to a low priority state for efficient multi-threading */
        sched_yield();
        
    }

    logger << "INFO: Joint ID " << _id << " angular control is now deactivated" << std::endl;
}


RoboticArm::RoboticArm(void) : _joints_nr(config::joints_nr)
{
    /* Initialize each joint objects with unique ID's */
    for(auto id = 0; id < _joints_nr; id++) {
        joints.push_back(new RoboticJoint(id));
    }
    logger << "INFO: Created a " << _joints_nr << " joints arm object" << std::endl;
}


RoboticArm::~RoboticArm(void)
{
    /* Call each of the joints destructors and stop any movement object */
    for(auto id = 0; id < _joints_nr; id++) {
        delete joints[id];
    }
}


void RoboticArm::Init(void)
{
    logger << __PRETTY_FUNCTION__ << std::endl;
    /* Perform the initialization for each of the joints */
    for(auto id = 0; id < _joints_nr; id++) {

        RoboticJoint * const joint = joints[id];
        
        double difference, min_speed = 0;
        const double epsilon = 0.000001;
        const double delta = 0.1;

        /* PHASE 0: */
        /* Calibrate each motor independently to find the minimum speed
         * value that produces real movement, due to rounding aritmethic
         * errors we use an epsilon comparision in order to see if the
         * value difference is more than that
         */
        joint->Movement->SetSpeed(0);
        joint->Movement->SetDirection(Motor::Direction::CCW);
        joint->Movement->Start();
        do {
            min_speed += delta;
            joint->Movement->SetSpeed(min_speed);
            auto old = joint->Position->GetAngle();
            usleep(100E03);
            difference = std::abs(joint->Position->GetAngle() - old);
        } while (difference < epsilon);
        min_speed -= delta;
        logger << "INFO: joint ID " << id << " min speed is ~" << min_speed << "%" << std::endl;
        joint->Movement->SetMinSpeed(min_speed);
        joint->Movement->Stop();

        /* PHASE I: */
        /* Get the rotors to a known position on a tight controlled loop 
         * due to rounding aritmethic errors, we use an epsilon comparision
         * in order to see if the values difference is less than it
         */
        joint->Movement->SetDirection(Motor::Direction::CW);
        //joint->Movement->SetSpeed(1);
        joint->Movement->Start();
        do {
            auto old = joint->Position->GetAngle();
            usleep(100E03);
            difference = std::abs(joint->Position->GetAngle() - old);
        } while (difference > epsilon);
        /* Reset the position coordinates, this is our new home position */
        joint->Movement->Stop();
        joint->SetZero();

        /* PHASE II: */
        /* Let the the joint correction control thread run and motors start-up */
        joint->Init();
        logger << "INFO: joint ID " << id << " was fully initialized" << std::endl;
        
        /* oxavelar: debugging single rotor for now */
        break;

    }
    
    logger << "INFO: Success, RoboticArm is now ready to operate!" << std::endl;
}


void RoboticArm::GetPosition(Point &pos)
{
    /* Temporary working matrix to fill sensor data */
    std::vector<double> theta;
    
    /* Fill our N joints angles in our temporary matrix in radians */
    for(auto id = 0; id < _joints_nr; id++) {
        theta.push_back( joints[id]->GetAngle() / 180 * M_PI );
    }

    /* Makes use of forward kinematics in order to get position */
    ForwardKinematics(pos, theta);
}


void RoboticArm::SetPosition(const Point &pos)
{
    /* Temporary working matrix to store our reference angles */
    std::vector<double> theta(_joints_nr);

    /* Makes use of inverse kinematics in order to set position */
    InverseKinematics(pos, theta);
    
    /* Update each of the joints their new reference angle */
    for(auto id = 0; id < _joints_nr; id++) {
        joints[id]->SetAngle( theta[id] * 180 / M_PI );
    }
}


void RoboticArm::ForwardKinematics(Point &pos, const std::vector<double> &theta)
{
    /* Temporary coordinate variable, to check for unsolvable solutions */
    Point tpos;

    /* Length of the links in meters, read only */
    const auto *L = &config::link_lengths[0];

    switch(theta.size())
    {
        case 1:
            tpos.x = L[0] * cos(theta[0]);
            tpos.y = L[0] * sin(theta[0]);
            tpos.z = 0;
            break;
        case 2:
            tpos.x = L[0] * cos(theta[0]) + L[1] * cos(theta[0] + theta[1]);
            tpos.y = L[0] * sin(theta[0]) + L[1] * sin(theta[0] + theta[1]);
            tpos.z = 0;
            break;
        default:
            /* oxavelar: To extend this to 3 dimensions for N joints */
            logger << "ERROR: Unable to calculate for more than 2 joints for now..." << std::endl;
            exit(-127);
            break;
    }

    /* Only update the target position if a solution in the 3D space was found */
    if (std::isnan(tpos.x) or std::isnan(tpos.y) or std::isnan(tpos.z)) {
        logger << "ERROR: Desired target position is not achievable by this robot" << std::endl;
    } else {
        pos = tpos;
    }
}


void RoboticArm::InverseKinematics(const Point &pos, std::vector<double> &theta)
{
    /* Length of the links in meters, read only */
    const auto *L = &config::link_lengths[0];

    switch(theta.size())
    {

        case 1:
            theta[0] = atan2(pos.y, pos.x);
        case 2:
            #define D ((pos.x*pos.x + pos.y*pos.y - L[0]*L[0] - L[1]*L[1]) / (2 * L[0] * L[1]))
            theta[1] = atan( sqrt(1 - D*D) / D );
            theta[0] = atan(pos.y / pos.x) - atan( (L[1] * sin(theta[1])) / (L[0] + L[1] * cos(theta[1])) );
            break;
        default:
            /* oxavelar: To extend this to 3 dimensions for N joints */
            logger << "ERROR: Unable to calculate for more than 2 joints for now..." << std::endl;
            exit(-127);
            break;
    }
}

