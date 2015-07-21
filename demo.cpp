#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <pthread.h>
#include "RoboticArm.h"


RoboticArm *RoboArm;


void _cleanup(int signum)
{
   std::cout << "\nINFO: Caught signal " << signum << std::endl;

   /* Delete all of the robotic-arm objects */
    delete RoboArm;

    exit(signum);
}


int main(void)
{
    /* Two joints robotic arm for the demo */
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);
    
    /* Higher priority for interrupt procesisng */
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };

    if( sched_setscheduler(0, SCHED_FIFO, &sp) != 0 ) {
        std::cout << "WARNING: Failed to increase process priority!" << std::endl;
    }

    RoboArm->Init();
    usleep(2E06);
    
    /* Input a curve or shape to the roboarm to draw it */
    for(;;) {
        RoboArm->UpdatePosition();
        usleep(3E06);
        std::cout << std::endl;
    }

    return EXIT_SUCCESS;
}

