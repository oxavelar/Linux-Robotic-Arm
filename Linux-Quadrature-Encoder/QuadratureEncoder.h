#pragma once
#include <iostream>
#include <chrono>
#include <atomic>
#include <stdint.h>
#include "../HighLatencyGPIO/GPIO.hh"

class QuadratureEncoder
{
    public:
        enum class Direction : int { CCW = -1, CW = 1 };

        explicit QuadratureEncoder(const int &pin_a, const int &pin_b);
        virtual ~QuadratureEncoder(void);
        
        void SetParameters(const int &segments);
        double GetDegrees(void);
        int GetPosition(void);
        std::chrono::nanoseconds GetPeriod(void);
        void SetZero(void);
        Direction GetDirection(void);

        void PrintStats(void);

    private:
        /* Pulse train inputs objects from the GPIO class */
        GPIO *_gpio_a, *_gpio_b;
        
        /* GPIO Interrupt routine user code */
        void _ISR_ChannelA(void);
        void _ISR_ChannelB(void);

        /* Callback references to be used by GPIO class */
        std::function<void(GPIO::Value)> _channel_a_callback;
        std::function<void(GPIO::Value)> _channel_b_callback;
        
        /* Quadrature Encoder Matrix for conversion
           http://letsmakerobots.com/content/how-use-quadrature-encoder */
        std::atomic_int _prev_packed_read_a, _prev_packed_read_b;
        const char _qem[16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

        /* Internal state variables */
        std::atomic_int _counter;
        std::chrono::nanoseconds _pulse_period_ns;
        std::atomic<Direction> _direction;
        
        /* How many counts are an actual revolution */
        int _segments_per_revolution;

        /* Used to keep track of the actual interrupt pulse-widths */
        void _TrackChannelPulseWidth(void);

        std::chrono::high_resolution_clock::time_point _isr_timestamp;

        /* Debug variables */
        std::atomic_ullong _channel_a_isr_cnt, _channel_b_isr_cnt;
};
