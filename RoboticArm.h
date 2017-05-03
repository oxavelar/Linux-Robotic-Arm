#pragma once
#include <atomic>
#include "Linux-DC-Motor/Motor.h"
#include "Linux-Quadrature-Encoder/QuadratureEncoder.h"
#include "Linux-Visual-Encoder/VisualEncoder.h"

#define epsilon (double)10E-09

/* Decimal spaces to trim for comparision tolerance*/
#define tolerance (double)1E-04
#define trim_precision (double)1E-07
#define trim(x) ((long long)std::round( x / trim_precision ) * (double)trim_precision)


class Point
{
    public:
        double x, y, z;
        bool operator==(const Point &p) {
            return ((std::abs(trim(p.x) - trim(x)) < tolerance) &&
                    (std::abs(trim(p.y) - trim(y)) < tolerance) &&
                    (std::abs(trim(p.z) - trim(z)) < tolerance));
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

