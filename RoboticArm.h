#pragma once
#include <iostream>
#include <stdint.h>
#include "Linux-DC-Motor/Motor.h"
#include "Linux-Quadrature-Encoder/QuadratureEncoder.h"


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

        /* Per joint position correction controls */
        void _AngularControl(void);
        std::thread _AutomaticControlThread;
};


class RoboticArm
{
    public:
        explicit RoboticArm(void);
        virtual ~RoboticArm(void);

        void Init(void);
        void DebugMessages(void);
        void GetPosition(Point &pos);
        void SetPosition(const Point &pos);

    private:
        int _joints_nr;
        std::vector<RoboticJoint*> joints;
};



