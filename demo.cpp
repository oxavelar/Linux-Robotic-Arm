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
        logger << "W: Failed to increase process priority!\n" << std::endl;
    }
}


void _cleanup(int signum)
{
    logger << "I: Caught signal " << signum << std::endl;

   /* Delete all of the robotic-arm objects */
    delete RoboArm;

    /* Finishes up gracefully the curses screen */
    sleep(5);
    endwin();

    logger << "I: Press any key to exit...\n" << std::endl;
    system("read -r REPLY");

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
    toolbox::ncurses_stream redirector_cout(std::cout);
    
#ifdef RT_PRIORITY
    SetProcessPriority(RT_PRIORITY);
#endif

    /* Please check RoboticArtm_Config.h for number of joints*/
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    RoboArm->Init();

    logger << "I: Press the arrow keys to control the robotic arm!" << std::endl << std::endl;

    for(;;) {

        /* Used for single line message */
        char buffer[80];        
        
        /* First obtain the actual coordinates of the robot, to move it at will */
        RoboArm->GetPosition(coordinates);

        /* Arrows will increase position by 1% increments in a x,y plane, uses curses library */
        WaitKeyPress(coordinates);

        /* Command the robot to a new position once that coordinates was updated */
        RoboArm->SetPosition(coordinates);
        
        sprintf(buffer, " x= %+3.4f | y= %+3.4f | z= %+3.4f", coordinates.x, coordinates.y, coordinates.z);
        logger << "I: Computed - " << buffer << std::endl;

        /* Updated coordinates and print it out */
        RoboArm->GetPosition(coordinates);

        sprintf(buffer, " x= %+3.4f | y= %+3.4f | z= %+3.4f", coordinates.x, coordinates.y, coordinates.z);
        logger << "I: Measured - " << buffer << std::endl;
        
    }

    return EXIT_SUCCESS;
}

