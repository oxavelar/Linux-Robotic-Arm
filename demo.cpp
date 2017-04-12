#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <boost/timer/timer.hpp>
#include <iomanip>
#include "toolbox.h"
#include "RoboticArm.h"
#include "RoboticArm_Config.h"


RoboticArm *RoboArm;
Point coordinates;

#ifdef RT_PRIORITY
void SetProcessPriority(const int &number)
{
    /* Higher priority for interrupt procesisng */
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

#ifdef NCURSES_SUPPORT
    /* Finishes up gracefully the curses screen */
    endwin();
    system("reset");
#endif
    
    /* Delete all of the robotic-arm objects */
    delete RoboArm;
    
    exit(signum);
}

#ifdef NCURSES_SUPPORT
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
#endif

#ifdef DIAGNOSTICS
void RunDiagnostics(RoboticArm *RoboArm, const long max_samples)
{
    /* Target vs Measured coordinate variables */
    Point t_coordinates, m_coordinates;

    logger << "I: Entering diagnostics mode for measuring latency" << std::endl;

    /* Initializing the rng seed */
    std::srand(std::time(0));

    for(auto s = 0; s < max_samples; s++) {

        /* Calculation for two random x,y parameters, this is hardcoded at the moment */
        t_coordinates.x = 0;
        t_coordinates.y = config::link_lengths[0] + config::link_lengths[1];
        t_coordinates.z = 0;

        /* Profiling with boost libraries to get cpu time and wall time */
        boost::timer::auto_cpu_timer *t = new boost::timer::auto_cpu_timer();

        /* Command the robot to a new position once that coordinates was updated */
        RoboArm->SetPosition(t_coordinates);

        /* Keep here until the robot reaches it's destination */
        do {
            RoboArm->GetPosition(m_coordinates);
        } while(t_coordinates != m_coordinates);
        
        /* Measurements get printed after this */
        delete t; 
    }

    logger << "I: Finished running diagnostics mode" << std::endl << std::endl;
}
#endif

void SPrintCoordinates(const Point &coordinates, char *buffer)
{
    sprintf(buffer, " x= %+3.4f | y= %+3.4f | z= %+3.4f", 
            coordinates.x, coordinates.y, coordinates.z);
}

int main(void)
{
#ifdef NCURSES_SUPPORT
    InitializeScreen();
    /* Redirect all of std::cout to a curses complaint window */
    toolbox::ncurses_stream redirector_cout(std::cout);
#endif
    
#ifdef RT_PRIORITY
    SetProcessPriority(RT_PRIORITY);
#endif

    /* Please check RoboticArtm_Config.h for number of joints*/
    RoboArm = new RoboticArm();
    
    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    RoboArm->Init();

#ifdef DIAGNOSTICS
    /* Perform N samples of measurements and print statistics */
    RunDiagnostics(RoboArm, 100);
#endif

    logger << "I: Press the arrow keys to control the robotic arm!" << std::endl << std::endl;

    for(;;) {

        /* Used for single line message */
        char buffer[80];

        /* Not used right now, but can be used for analytics */
        /* NOTE: This is buggy right now */

        auto load = toolbox::get_cpu_load();
        logger << "I: CPU Utilization - " << std::fixed << std::setw(11) << std::setprecision(6)
                                          << load << std::endl;

        /* First obtain the actual coordinates of the robot, to move it at will */
        RoboArm->GetPosition(coordinates);

#ifdef NCURSES_SUPPORT
        /* Arrows will increase position by 1% increments in a x,y plane, uses curses library */
        WaitKeyPress(coordinates);

        /* Command the robot to a new position once that coordinates was updated */
        RoboArm->SetPosition(coordinates);
        
	SPrintCoordinates(coordinates, buffer);
        logger << "I: Computed - " << buffer << std::endl;
#else
        /* Press [ENTER] to continue... */
        std::cin.ignore();
#endif

        /* Updated coordinates and print it out */
        RoboArm->GetPosition(coordinates);

	SPrintCoordinates(coordinates, buffer);
        logger << "I: Measured - " << buffer << std::endl;
        
    }

    return EXIT_SUCCESS;
}

