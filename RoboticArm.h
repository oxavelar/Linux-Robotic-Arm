#pragma once
#include <iostream>
#include <stdint.h>
#include "Linux-DC-Motor/Motor.h"
#include "Linux-Quadrature-Encoder/QuadratureEncoder.h"


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

    private:
        int _joints_nr;
        std::vector<RoboticJoint*> joints;
};



