#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <getopt.h>
#include <sys/mman.h>
#include <boost/timer/timer.hpp>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include "../toolbox.h"
#include "../RoboticArm.h"
#include "../RoboticArm_Config.h"


RoboticArm *RoboArm;
std::ofstream *outfile;

/* Global command line knobs */
std::string cl_option_filename;

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

    outfile->close();

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

void PrintUsage()
{
    const std::string usage                                   \
("                                                          \n\
Usage: linux-robotic-arm-recorder.app -f FILE               \n\
Used to record the trajectory of a robotic arm.             \n\
                                                            \n\
    -f,--file=     Trajectory file to playback              \n\
    -h,--help      Prints the usage and exit (this screen)  \n\
                                                            \n\
                                                            \n\
Example:                                                    \n\
linux-robotic-arm-recorder.app -f trajectory.rec            \n\
");
    std::cerr << usage << std::endl;
    exit(EXIT_FAILURE);
}

void ProcessCLI(int argc, char *argv[])
{
    int c, option_index = 0;

    struct option long_options[] = {
        { "file"    , required_argument ,0, 'f'},
        { "help"    , no_argument       ,0, 'h'},
        { 0         , 0                 ,0,  0 }
    };

    if (argc < 2)
        PrintUsage();

    while ((c = getopt_long(argc, argv, "f:h", long_options, &option_index)) != -1)
        switch(c) {

            case 'f':
                cl_option_filename.assign(optarg);
                break;

            case 'h':
            case '?':
            default:
                PrintUsage();

        }
}

int main(int argc, char *argv[])
{
    /* Process the trajectory filename and arguments */
    ProcessCLI(argc, argv);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Please check RoboticArtm_Config.h for number of joints*/
    //RoboArm = new RoboticArm();
    
    /* File that we will be writing to */
    outfile = new std::ofstream(cl_option_filename, std::ofstream::binary);

    if(outfile == NULL) {
        logger << "E: Failed to write the trajectory file" << std::endl;
        exit(-73);
    }

    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    //RoboArm->Init();

    /* Disable automatic control to move it with our hands to train it */

    logger << "I: Recording trajectory file: \"" << cl_option_filename << "\"" << std::endl;
    logger << "I: You can now begin to move the robot" << std::endl;
    logger << "I: Press <Ctrl-C> to stop recording" << std::endl;

    for(;;) {
        usleep(500E03);
        outfile->write("ping pong\n", 20);
    }
    
    _cleanup(EXIT_SUCCESS);

    return EXIT_SUCCESS;
}

