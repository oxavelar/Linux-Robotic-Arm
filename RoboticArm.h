#pragma once
#include <iostream>
#include <stdint.h>
#include "Linux-Quadrature-Encoder/QuadratureEncoder.h"

class RoboticArm
{
    public:
        RoboticArm(const uint16_t &joints_nr);
        virtual ~RoboticArm(void);

        void UpdatePosition(void);

    private:
        uint8_t _joints_nr;

#ifndef VISUAL_ENCODER
        std::vector<QuadratureEncoder*> angular_joints;
#else
        std::vector<VisualeEncoder*> angular_joints;
#endif
};
