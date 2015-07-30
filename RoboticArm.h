#pragma once
#include <iostream>
#include <stdint.h>
#include "Linux-DC-Motor/Motor.h"
#include "Linux-Quadrature-Encoder/QuadratureEncoder.h"

using Point = std::vector<float>;


class RoboticJoint
{
    public:
        explicit RoboticJoint(const int &id);
        virtual ~RoboticJoint(void);

        /* Quadrature encoders + DC motors */
        QuadratureEncoder* Position;
        Motor* Movement;

    private:
        const int _id;
        
};


class RoboticArm
{
    public:
        explicit RoboticArm(void);
        virtual ~RoboticArm(void);

        void Init();
        void DemoCircle(void);
        void UpdatePosition(void);
        Point GetPosition(void);
        void SetPosition(Point);

    private:
        int _joints_nr;
        std::vector<RoboticJoint*> joints;
};



