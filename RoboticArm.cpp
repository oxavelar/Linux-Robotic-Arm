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
 * http://www.hessmer.org/uploads/RobotArm/Inverse%2520Kinematics%2520for%2520Robot%2520Arm.pdf
 *
 * Authors: Omar X. Avelar & Juan C. Razo
 *
 */

#include <iostream>
#include <sstream>
#include <cmath>
#include <atomic>
#include <chrono>
#include <thread>
#include "toolbox.h"
#include "RoboticArm.h"
#include "RoboticArm_Config.h"


RoboticJoint::RoboticJoint(const int &id) :
    _id(id),
    _reference_angle(0),
    _control_thread_stop_event(false)
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
    Movement->Stop();

    /* Stop the automatic control loop thread */
    if (AutomaticControlThread.joinable()) {
        _control_thread_stop_event = true;
        AutomaticControlThread.join();
    }

    delete Position;
    delete Movement;
}


void RoboticJoint::Init(void)
{
    /* Set the motors running, so the control loop can work on it */
    Movement->SetSpeed(0);
    Movement->Start();
    logger << "I: Joint ID " << _id << " is in our home position" << std::endl;
    /* Register our control thread */
    AutomaticControlThread = std::thread(&RoboticJoint::AngularControl, this);
}


double RoboticJoint::GetAngle(void)
{
    /* Reads the internal variable, this is used by 
     * the control loop thread, so just reading it is
     * enough information of where the joint is */
    return _reference_angle;
}


void RoboticJoint::SetAngle(const double &theta)
{
    /* Update the internal variable, the control loop
     * will take charge of getting us here eventually 
     * theta is in radians so converting from 0 to 360 */
    _reference_angle = (theta >= 0 ? theta : (2 * M_PI + theta)) * 180.0 / M_PI;
    _reference_angle = std::fmod(_reference_angle, 360.0);
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
    logger << "I: Joint ID " << _id << " angular control is now active" << std::endl;

    while(!_control_thread_stop_event) {
        
        /* Consists of the interaction between position & movement */
        const auto k = 8.00;
        /* Internal refernces are in degrees no conversion at all */
        const auto actual_angle = std::fmod(Position->GetAngle(), 360.0);
        const auto error_angle = actual_angle - _reference_angle;
        
        /* Sign dictates the direction of movement */
        if (error_angle >= 0)    Movement->SetDirection(Motor::Direction::CW);
        else                     Movement->SetDirection(Motor::Direction::CCW);
        
        /* Store the motor control value to the movement function */
        Movement->SetSpeed( k * std::abs(error_angle) );
        
#if (DEBUG_LEVEL >= 10)
        logger << "D: Joint ID " << _id << " actual=" << actual_angle << std::endl;
        logger << "D: Joint ID " << _id << " reference=" << _reference_angle << std::endl;
        logger << "D: Joint ID " << _id << " error=" << error_angle << std::endl;
        logger << "D: Joint ID " << _id << " computed speed=" << k * std::abs(error_angle) << "%" << std::endl;
        logger << "D: Joint ID " << _id << " measured speed=" << Movement->GetSpeed() << "%" << std::endl;
        logger << std::endl;
#endif
        
        /* Send this task to a low priority state for efficient multi-threading */
        sched_yield();
        
    }

    logger << "I: Joint ID " << _id << " angular control is now deactivated" << std::endl;
}


RoboticArm::RoboticArm(void) : _joints_nr(config::joints_nr)
{
    /* Initialize each joint objects with unique ID's */
    for(auto id = 0; id < _joints_nr; id++) {
        joints.push_back(new RoboticJoint(id));
    }
    logger << "I: Created a " << _joints_nr << " joints arm object" << std::endl;
}


RoboticArm::~RoboticArm(void)
{
    /* Call each of the joints destructors and stop any movement object */
    for(auto id = 0; id < _joints_nr; id++) {
        delete joints[id];
    }
}


void RoboticArm::CalibrateMovement(void)
{
    double difference;
    const double delta = 0.05;

    /* Perform the initialization for each of the joints */
    for(auto id = 0; id < _joints_nr; id++) {

        RoboticJoint * const joint = joints[id];

        /* Calibrate each motor independently to find the minimum speed
         * value that produces real movement, due to rounding aritmethic
         * errors we use an epsilon comparision in order to see if the
         * value difference is more than that
         */
        double min_speed = 0;
        joint->Movement->SetDirection(Motor::Direction::CCW);

        /* Coarse tuning, aproximate where the threshold movement is */
        do {
            min_speed += delta;

            /* Make sure we have not reached 100% */
            if((int)min_speed == 100) {
                logger << "E: Joint ID " << id << " is unable to move or detect movement!" << std::endl;
                exit(-99);
            }
            
            joint->Movement->SetSpeed(min_speed);
            auto old = joint->Position->GetAngle();
            joint->Movement->Start();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            joint->Movement->Stop();
            difference = std::abs(joint->Position->GetAngle() - old);
            
        } while(difference < epsilon);
        
        joint->Movement->SetDirection(Motor::Direction::CW);
        /* Fine tuning, go back by delta squared to where we stop moving in steady state */
        do {

            /* Make sure we have not reached 0% + delta */
            if((delta + epsilon) > min_speed) {
                logger << "E: Joint ID " << id << " is unable to stop at 0%!" << std::endl;
                exit(-100);
            }

            min_speed -= (delta * delta);
            joint->Movement->SetSpeed(min_speed);
            auto old = joint->Position->GetAngle();
            joint->Movement->Start();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            joint->Movement->Stop();
            difference = std::abs(joint->Position->GetAngle() - old);
            
        } while(difference < epsilon);
        
        logger << "I: Joint ID " << id << " min speed found for movement is ~" << min_speed << "%" << std::endl;
        logger << "I: Joint ID " << id << " set speed remap for 0% to 100% values" << std::endl;
        
        /* Now we can have a range from minimum speed to full */
        joint->Movement->ApplyRangeLimits(min_speed, 100);
    }
}


void RoboticArm::CalibratePosition(void)
{
    double difference;

    /* Perform the initialization for each of the joints */
    for(auto id = 0; id < _joints_nr; id++) {

        RoboticJoint * const joint = joints[id];
       
        /* Get the rotors to a known position on a tight controlled loop 
         * due to rounding aritmethic errors, we use an epsilon comparision
         * in order to see if the values difference is less than it
         */
        joint->Movement->SetDirection(Motor::Direction::CW);
        joint->Movement->SetSpeed(100);
        do {
            
            auto old = joint->Position->GetAngle();
            joint->Movement->Start();
            /* Must account for turn off and turn on delays, use bigger delay */
            std::this_thread::sleep_for(std::chrono::nanoseconds(1));
            joint->Movement->Stop();
            difference = std::abs(joint->Position->GetAngle() - old);
            
        } while(difference >= epsilon);

        /* Reset the position coordinates, this is our new home position */
        joint->SetZero();
        
    }
}


void RoboticArm::Init(void)
{
    /* Overall robot calibration */
    CalibrateMovement();
    CalibratePosition();
    
    /* Perform the initialization for each of the joints */
    for(auto id = 0; id < _joints_nr; id++) {

        RoboticJoint * const joint = joints[id];
        
        /* Let the the joint correction control thread run and motors start-up */
        joint->Init();

    }

    logger << "I: Robot was successfully initialized" << std::endl;
}


void RoboticArm::EnableTrainingMode(void)
{  
    /* Kill off the automatic control for each of the joints */
    for(auto id = 0; id < _joints_nr; id++) {

        RoboticJoint * const joint = joints[id];
        
        /* Kill off the movement */
        joint->Movement->Stop();

    }

    logger << "I: Robot was put in training mode for manual override" << std::endl;
}


void RoboticArm::GetPosition(Point &pos)
{
    /* Temporary working matrix to fill sensor data */
    std::vector<double> theta;
    
    /* Fill our N joints angles in our temporary matrix in radians */
    for(auto id = 0; id < _joints_nr; id++) {
        theta.push_back( joints[id]->GetAngle() / 180.0 * M_PI );
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
        joints[id]->SetAngle(theta[id]);
    }
}


void RoboticArm::SetPositionSync(const Point &pos)
{
    /* Synchronous version of SetPosition */
    Point measured, target = pos;

    SetPosition(target);

    /* Stay here until the robot reaches its destination */
    do {
        GetPosition(measured);
    } while(target != measured);
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
            tpos.x = L[0] * std::cos(theta[0]);
            tpos.y = L[0] * std::sin(theta[0]);
            tpos.z = 0;
            break;
        case 2:
            tpos.x = L[0] * std::cos(theta[0]) + L[1] * std::cos(theta[0] + theta[1]);
            tpos.y = L[0] * std::sin(theta[0]) + L[1] * std::sin(theta[0] + theta[1]);
            tpos.z = 0;
            break;
        default:
            /* oxavelar: To extend this to 3 dimensions for N joints */
            logger << "E: Unable to calculate for more than 2 joints for now..." << std::endl;
            exit(-127);
            break;
    }

    /* Only update the target position if a solution in the 3D space was found */
    if (std::isnan(tpos.x) or std::isnan(tpos.y) or std::isnan(tpos.z)) {
        logger << "E: Actual position should not be achievable by this robot" << std::endl;
    } else {
        pos = tpos;
    }
}


void RoboticArm::InverseKinematics(const Point &pos, std::vector<double> &theta)
{
    /* Length of the links in meters, read only */
    const auto *L = &config::link_lengths[0];

    /* Backup our angles */
    const std::vector<double> theta_backup = theta;

    switch(theta.size())
    {

        case 1:
            theta[0] = std::atan2(pos.y, pos.x);
        case 2:
            #define D ((pos.x*pos.x + pos.y*pos.y - L[0]*L[0] - L[1]*L[1]) / (2 * L[0] * L[1]))
            theta[1] = std::atan2( 1 - (D*D), D);
            theta[0] = std::atan2(pos.y, pos.x) - std::atan2( (L[1] * std::sin(theta[1])), (L[0] + L[1] * std::cos(theta[1])) );
            break;
        default:
            /* oxavelar: To extend this to 3 dimensions for N joints */
            logger << "E: Unable to calculate for more than 2 joints for now..." << std::endl;
            exit(-127);
            break;
    }

    /* Verify that each solved angle is a valid number before setting things up, abort and log */
    for(auto id = 0; id < _joints_nr; id++) {
        if(std::isnan(theta[id])) {
            logger << "E: Desired target position is not achievable by this robot" << std::endl;
            theta = theta_backup;
            break;
        }
    }
}

