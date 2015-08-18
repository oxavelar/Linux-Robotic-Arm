#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <chrono>
#include <ctime>
#include <string>
#include <curses.h>
#include "RoboticArm.h"
#include "RoboticArm_Config.h"


RoboticArm *RoboArm;
Point coordinates;


void _cleanup(int signum)
{
    std::cout << "\nINFO: Caught signal " << signum << std::endl;

    /* Finishes up gracefully the curses screen */
    endwin();

   /* Delete all of the robotic-arm objects */
    delete RoboArm;

    exit(signum);
}


const std::string timestamp(void)
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%X", &tstruct);

    return buf;
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
    /* Two joints robotic arm for the demo, please check RoboticArtm_Config.h */
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    /* Let curses know we just want keypads for control */
    newterm(NULL, stdin, stdout);
    nonl();
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
    refresh();

#ifdef RT_PRIORITY
    /* Higher priority for interrupt procesisng */
    /* https://rt.wiki.kernel.org/index.php/HOWTO:_Build_an_RT-application */
    struct sched_param sp = { .sched_priority = 99 };
    if( sched_setscheduler(0, SCHED_FIFO, &sp) != 0 ) {
        printw("Linux-Robotic-Arm: %s: WARNING: Failed to increase process priority!\n\n", timestamp().c_str());
    }
#endif

    RoboArm->Init();
    usleep(2E06);

    
    /* Input a curve or shape to the roboarm to draw it */
    for(;;) {

        RoboArm->GetPosition(coordinates);
        
        printw("Linux-Robotic-Arm: %s: INFO: x= %+8.9f | y= %+8.9f | z= %+8.9f \n",
                timestamp().c_str(), coordinates.x, coordinates.y, coordinates.z);

        /* Arrow keys will increase position by 1% distance increments in a x,y plane, uses curses library */
        WaitKeyPress(coordinates);
      
        /* Command the robot to a new position once that coordinates was updated */
        //RoboArm->SetPosition(coordinates);

    }

    return EXIT_SUCCESS;
}

