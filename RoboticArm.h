#pragma once
#include <atomic>
#include "Linux-DC-Motor/Motor.h"
#include "Linux-Quadrature-Encoder/QuadratureEncoder.h"
#include "Linux-Visual-Encoder/VisualEncoder.h"

#define epsilon (double)10E-09
#define precision (double)1E-03
#define roundtr(x) (std::round( x / precision ) * precision)

class Point
{
    public:
        double x, y, z;
        bool operator==(const Point &p) {
            return ((std::abs(roundtr(p.x) - roundtr(x)) < epsilon) &&
                    (std::abs(roundtr(p.y) - roundtr(y)) < epsilon) &&
                    (std::abs(roundtr(p.z) - roundtr(z)) < epsilon));
        };
        bool operator!=(const Point &p) { return !(*this == p); };
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
        std::atomic<double> _reference_angle;

        /* Per joint position correction control */
        void AngularControl(void);
        std::thread AutomaticControlThread;
        std::atomic<bool> _control_thread_stop_event;
};


class RoboticArm
{
    public:
        explicit RoboticArm(void);
        virtual ~RoboticArm(void);

        void Init(void);
        void GetPosition(Point &pos);
        void SetPosition(const Point &pos);

        void ForwardKinematics(Point &pos, const std::vector<double> &theta);
        void InverseKinematics(const Point &pos, std::vector<double> &theta);

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
};

