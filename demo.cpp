#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <string>
#include "RoboticArm.h"


RoboticArm *RoboArm;
Point actual_coordinates, reference_coordinates;


void _cleanup(int signum)
{
   std::cout << "\nINFO: Caught signal " << signum << std::endl;

   /* Delete all of the robotic-arm objects */
    delete RoboArm;

    exit(signum);
}


const std::string datetime(void)
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}


int main(void)
{
    /* Two joints robotic arm for the demo */
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

#ifdef RT_PRIORITY
    /* Higher priority for interrupt procesisng */
    /* https://rt.wiki.kernel.org/index.php/HOWTO:_Build_an_RT-application */
    struct sched_param sp = { .sched_priority = 99 };
    if( sched_setscheduler(0, SCHED_FIFO, &sp) != 0 ) {
        std::cout << "WARNING: Failed to increase process priority!" << std::endl;
    }
#endif

    RoboArm->Init();
    usleep(2E06);

    
    /* Input a curve or shape to the roboarm to draw it */
    for(;;) {
        RoboArm->GetPosition(actual_coordinates);
        
        /* fprintf is used in our demo app, elsewhere use streams */
        fprintf(stdout, "Linux-Robotic-Arm: info: %s - x: %4.9f | y: %4.9f | z: %4.9f \n",
                datetime().c_str(), actual_coordinates.x, actual_coordinates.y, actual_coordinates.z);

        /* Command the robot to a new position, should not move... */
        reference_coordinates = actual_coordinates;
        RoboArm->SetPosition(reference_coordinates);

        usleep(3E06);
    }

    return EXIT_SUCCESS;
}

