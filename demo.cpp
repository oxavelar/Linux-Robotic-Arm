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


void SetProcessPriority(const int &number)
{
    /* Higher priority for interrupt procesisng */
    /* https://rt.wiki.kernel.org/index.php/HOWTO:_Build_an_RT-application */
    struct sched_param sp = { .sched_priority = number };
    if( sched_setscheduler(0, SCHED_FIFO, &sp) != 0 ) {
        logger << "WARNING: Failed to increase process priority!\n" << std::endl;
    }
}


void _cleanup(int signum)
{
    std::cout << "\nINFO: Caught signal " << signum << std::endl;

    /* Finishes up gracefully the curses screen */
    endwin();

   /* Delete all of the robotic-arm objects */
    delete RoboArm;

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


int main(void)
{
    InitializeScreen();
    /* Redirect all of std::cout to a curses complaint window */
    toolbox::ncurses_stream redirector(std::cout);
    
#ifdef RT_PRIORITY
    SetProcessPriority(90);
#endif

    /* Please check RoboticArtm_Config.h for number of joints*/
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    RoboArm->Init();
    
    usleep(5E06);

    logger << "INFO: Press the arrow keys to control the robotic arm!" << std::endl << std::endl;

    for(;;) {

        /* Arrow keys will increase position by 1% distance increments in a x,y plane, uses curses library */
        WaitKeyPress(coordinates);

        RoboArm->GetPosition(coordinates);
        
        char buffer[80];
        sprintf(buffer, "x= %+8.9f | y= %+8.9f | z= %+8.9f", coordinates.x, coordinates.y, coordinates.z);
        logger << "INFO: " << buffer << std::endl;;
      
        /* Command the robot to a new position once that coordinates was updated */
        RoboArm->SetPosition(coordinates);
    }

    return EXIT_SUCCESS;
}

