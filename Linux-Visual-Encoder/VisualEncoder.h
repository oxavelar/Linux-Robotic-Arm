#pragma once
#include <atomic>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


class VisualEncoder
{
    public:
        explicit VisualEncoder(const int &port);
        virtual ~VisualEncoder(void);

        double GetAngle(void);
        void SetZero(void);
        Direction GetDirection(void);

    private:
        /* webcam port used in opencv */
        const int _port;
};

