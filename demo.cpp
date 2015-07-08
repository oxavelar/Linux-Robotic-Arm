#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include "RoboticArm.h"

RoboticArm *RoboArm;

void cleanup(void)
{
   /* Delete all of the robotic-arm objects */
    delete RoboArm;
}

int main(int argc, char *argv[])
{
    RoboArm = new RoboticArm();
    

    /* Register a signal handler */
    //signal(SIGINT, cleanup);

    for(;;) {
        usleep(200000);
    }

    return EXIT_SUCCESS;
}
