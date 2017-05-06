#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <boost/timer/timer.hpp>
#include <iomanip>
#include <random>
#include "../toolbox.h"
#include "../RoboticArm.h"
#include "../RoboticArm_Config.h"


RoboticArm *RoboArm;
Point coordinates;

#ifdef RT_PRIORITY
void SetProcessPriority(const int &number)
{
    /* https://rt.wiki.kernel.org/index.php/HOWTO:_Build_an_RT-application */
    struct sched_param sp = { .sched_priority = number };
    if( sched_setscheduler(0, RT_POLICY, &sp) != 0 ) {
        logger << "W: Failed to increase process priority!\n" << std::endl;
    }
}
#endif

void _cleanup(int signum)
{
    logger << "I: Caught signal " << signum << std::endl;

    munlockall();

    /* Delete all of the robotic-arm objects */
    delete RoboArm;
    
    exit(signum);
}

void SPrintCoordinates(const Point &coordinates, char *buffer)
{
    sprintf(buffer, " x= %+2.5f | y= %+2.5f | z= %+2.5f", 
            coordinates.x, coordinates.y, coordinates.z);
}

int main(void)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Preload the input file, and start loading it in memory */
    

    /* Please check RoboticArtm_Config.h for number of joints*/
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    RoboArm->Init();

    for(;;) {

        /* Used for single line message */
        char buffer[80];

        /* First obtain the actual coordinates of the robot, to move it at will */
        RoboArm->GetPosition(coordinates);

        SPrintCoordinates(coordinates, buffer);
        logger << "I: Moving to - " << buffer << std::endl;

        /* Command the robot to a new position */
        RoboArm->SetPositionSync(coordinates);
       
    }

    return EXIT_SUCCESS;
}

