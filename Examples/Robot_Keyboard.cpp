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


std::unique_ptr<RoboticArm> RoboArm;
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

    /* Finishes up gracefully the curses screen */
    endwin();
    system("reset");
    
    exit(signum);
}

void InitializeScreen(void)
{
    initscr();
    scrollok(stdscr, TRUE);
    keypad(stdscr, TRUE);
    refresh();
}

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

void SPrintCoordinates(const Point &coordinates, char *buffer)
{
    sprintf(buffer, " x= %+2.5f | y= %+2.5f | z= %+2.5f", 
            coordinates.x, coordinates.y, coordinates.z);
}

int main(void)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);

    InitializeScreen();
    /* Redirect all of std::cout to a curses complaint window */
    toolbox::ncurses_stream redirector_cout(std::cout);
    
#ifdef RT_PRIORITY
    SetProcessPriority(RT_PRIORITY);
#endif

    /* Please check RoboticArtm_Config.h for number of joints*/
    RoboArm = std::unique_ptr<RoboticArm>(new RoboticArm());
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    RoboArm->Init();

    logger << "I: Press the arrow keys to control the robotic arm!" << std::endl << std::endl;

    for(;;) {

        /* Used for single line message */
        char buffer[80];

        /* Not used right now, but can be used for analytics */

        auto load = toolbox::get_cpu_load();
        logger << "I: CPU Utilization - " << std::fixed << std::setw(11) << std::setprecision(6)
                                          << load << std::endl;

        /* First obtain the actual coordinates of the robot, to move it at will */
        RoboArm->GetPosition(coordinates);

        /* Arrows will increase position by 1% increments in a x,y plane, uses curses library */
        WaitKeyPress(coordinates);

        /* Command the robot to a new position once that coordinates was updated */
        RoboArm->SetPosition(coordinates);
        
        /* Updated coordinates and print it out */
        RoboArm->GetPosition(coordinates);

        SPrintCoordinates(coordinates, buffer);
        logger << "I: Measured - " << buffer << std::endl;
        
    }

    return EXIT_SUCCESS;
}

