#pragma once
#include <iostream>
#include <stdint.h>

class QuadratureEncoder
{
    public:
        QuadratureEncoder(void);
        ~QuadratureEncoder(void);

        void Init(void);
        void Start(void);
        void Stop(void);

    private:
        /* Both pulse train inputs */
        uint32_t channel_a;
        uint32_t channel_b;

        /* Local counter variables */
        uint32_t counter_val;

};
