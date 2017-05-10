#pragma once
#include <iostream>
#include <atomic>
#include <chrono>
#include "../HighLatencyGPIO/GPIO.hh"


class QuadratureEncoder
{
    public:
        enum class Direction : int { CCW = -1, CW = 1 };

        explicit QuadratureEncoder(const int &pin_a, const int &pin_b, const int &rate=4);
        virtual ~QuadratureEncoder(void);
        
        double GetAngle(void);
        void SetZero(void);
        Direction GetDirection(void);
        void SetParameters(const int &segments);
        std::chrono::nanoseconds GetPeriod(void);

    private:
        /* Pulse train inputs objects from the GPIO class */
        std::unique_ptr<GPIO> _gpio_a;
        std::unique_ptr<GPIO> _gpio_b;
        
        /* GPIO Interrupt routine user code */
        void ISR_ChannelA(void);
        void ISR_ChannelB(void);

        /* Callback references to be used by GPIO class */
        std::function<void(GPIO::Value)> _channel_a_callback;
        std::function<void(GPIO::Value)> _channel_b_callback;
        
        /* Quadrature Encoder Matrix for conversion
           http://letsmakerobots.com/content/how-use-quadrature-encoder

            2x Rate: [3,2,3,2...] or [2,3,2,3...]
            4x Rate: [0,1,3,2...] or [0,2,3,1...] 

            Depending on the direction, they are packed signal values, can be seen
            as an array of 4 bit values, starting from one channel extending to the
            other.

           Note: If a value of 'x' is read it means the code is too slow!
        */
        inline void GPIO_DataProcess(void);

        std::atomic<int> _prev_packed_read;
        const signed char _qem[16] = {0,-1,1,'x',1,0,'x',-1,-1,'x',0,1,'x',1,-1,0};

        /* Internal state variables */
        std::atomic<long> _counter;
        std::chrono::nanoseconds _pulse_period_ns;
        std::atomic<Direction> _direction;
        
        /* If we are in 1x, 2x or 4x rates */
        const int _encoder_rate;

        /* How many counts are an actual revolution */
        int _segments_per_revolution;

        /* Used to keep track of the actual interrupt pulse-widths */
        std::atomic<char> _internal_toggle;
        void TrackChannelPulseWidth(void);

        std::chrono::high_resolution_clock::time_point _isr_timestamp;

#ifdef DEBUG
        std::atomic<unsigned long long> _channel_a_isr_count, _channel_b_isr_count;
        std::atomic<unsigned long long> _gpio_processing_error_count;
        void PrintDebugStats(void);
#endif
};
