#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include "toolbox.h"
#include "RoboticArm.h"
#include "RoboticArm_Config.h"


RoboticArm *RoboArm;
Point coordinates;


void _cleanup(int signum)
{
    std::cout << "\nINFO: Caught signal " << signum << std::endl;

    /* Finishes up gracefully the curses screen */
    //endwin();

   /* Delete all of the robotic-arm objects */
    delete RoboArm;

    exit(signum);
}




#if 0
void WaitKeyPress(Point &coordinates)
{
    int key;
    key = getch();
    
    switch(key)
    {
        case KEY_LEFT:
            coordinates.x -= (1 * config::link_lengths[0] / 100);
            break;
        case KEY_RIGHT:
            coordinates.x += (1 * config::link_lengths[0] / 100);
            break;
        case KEY_UP:
            coordinates.y += (1 * config::link_lengths[0] / 100);
            break;
        case KEY_DOWN:
            coordinates.y -= (1 * config::link_lengths[0] / 100);
            break;
        default:
            break;
    }
}
#endif


int main(void)
{
#ifdef RT_PRIORITY
    /* Higher priority for interrupt procesisng */
    /* https://rt.wiki.kernel.org/index.php/HOWTO:_Build_an_RT-application */
    struct sched_param sp = { .sched_priority = 50 };
    if( sched_setscheduler(0, SCHED_RR, &sp) != 0 ) {
        logger << "WARNING: Failed to increase process priority!\n" << std::endl;
    }
#endif

    /* Please check RoboticArtm_Config.h for number of joints*/
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    RoboArm->Init();
    usleep(1E06);

    /* Input a curve or shape to the roboarm to draw it */
    for(;;) {

        RoboArm->GetPosition(coordinates);
        
        char buffer[80];
        sprintf(buffer, "x= %+8.9f | y= %+8.9f | z= %+8.9f \n", coordinates.x, coordinates.y, coordinates.z);
        logger << "INFO: " << buffer;

        /* Arrow keys will increase position by 1% distance increments in a x,y plane, uses curses library */
        //WaitKeyPress(coordinates);
      
        /* Command the robot to a new position once that coordinates was updated */
        RoboArm->SetPosition(coordinates);

        usleep(2E6);

    }

    return EXIT_SUCCESS;
}

