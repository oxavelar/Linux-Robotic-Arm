#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include "RoboticArm.h"


RoboticArm *RoboArm;


void _cleanup(int signum)
{
   std::cout << "INFO: Caught signal " << signum << std::endl;

   /* Delete all of the robotic-arm objects */
    delete RoboArm;

    exit(signum);
}


int main(int argc, char *argv[])
{
    /* Two joints robotic arm for the demo */
    RoboArm = new RoboticArm(2);
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    /* Input a curve or shape to the roboarm to draw it */
    for(;;) {
        //RoboArm->Circle();
        RoboArm->UpdatePosition();
        usleep(800000);
        std::cout << std::endl;
    }

    return EXIT_SUCCESS;
}

