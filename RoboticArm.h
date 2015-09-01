#pragma once
#include <atomic>
#include "Linux-DC-Motor/Motor.h"
#include "Linux-Quadrature-Encoder/QuadratureEncoder.h"
#include "Linux-Visual-Encoder/VisualEncoder.h"


class Point
{
    public:
        double x, y, z;
};


class RoboticJoint
{
    public:
        explicit RoboticJoint(const int &id);
        virtual ~RoboticJoint(void);

        void Init(void);
        double GetAngle(void);
        void SetAngle(const double &theta);
        void SetZero(void);


        /* Quadrature encoders + DC motors */
        QuadratureEncoder* Position;
        Motor* Movement;

    private:
        const int _id;
        double _reference_angle;

        /* Per joint position correction control */
        void AngularControl(void);
        std::thread AutomaticControlThread;
        bool _control_thread_stop_event;
};


class RoboticArm
{
    public:
        explicit RoboticArm(void);
        virtual ~RoboticArm(void);

        void Init(void);
        void GetPosition(Point &pos);
        void SetPosition(const Point &pos);

    private:
        const int _joints_nr;
        /* A container of joints form a chain, 
         * joints[0] = root 
         * joints[1] = node1
         * :
         * joints[n] = nodeN
         */
        std::vector<RoboticJoint*> joints;

        void CalibrateMovement(void);
        void CalibratePosition(void);

        void ForwardKinematics(Point &pos, const std::vector<double> &theta);
        void InverseKinematics(const Point &pos, std::vector<double> &theta);
};



