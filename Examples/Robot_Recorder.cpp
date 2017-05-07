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

#define RECORD_RATE_HZ 20
std::unique_ptr<std::ofstream> outfile;

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
    
    outfile->flush();
    outfile->close();

    munlockall();
    
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
    std::unique_ptr<RoboticArm> RoboArm;
    Point coordinates;
    double timestamp = 0.0;

    /* Used for single line messages */
    char buffer[80];

    /* Process the trajectory filename and arguments */
    ProcessCLI(argc, argv);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Please check RoboticArtm_Config.h for number of joints*/
    RoboArm = std::unique_ptr<RoboticArm>(new RoboticArm());
    
    /* File that we will be writing to */
    outfile = std::unique_ptr<std::ofstream>(new std::ofstream(cl_option_filename,
                                                               std::ios::binary |
                                                               std::ios::trunc));

    if(outfile == NULL) {
        logger << "E: Failed to write the trajectory file" << std::endl;
        exit(-73);
    }

    /* Register a signal handler to exit gracefully */
    signal(SIGINT, _cleanup);

    RoboArm->Init();

    /* Disable automatic control to move it with our hands to train it */
    RoboArm->EnableTrainingMode();

    logger << "I: Recording trajectory file: \"" << cl_option_filename << "\"" << std::endl;
    logger << "I: You can now begin to move the robot" << std::endl;
    logger << "I: Press <Ctrl-C> to stop recording" << std::endl;

    for(;;) {

        /* Update the position and time stamp before we write it down */
        RoboArm->GetPosition(coordinates);
        timestamp += (1 / (double)RECORD_RATE_HZ);

        /* Format is x y z timestamp */
        sprintf(buffer, "%2.8f %2.8f %2.8f %2.9f",
                coordinates.x, coordinates.y, coordinates.z, timestamp);

        /* Write into the file */
        *outfile << buffer << std::endl;

        usleep((1 / (double)RECORD_RATE_HZ) * 1E06);

    }
    
    _cleanup(EXIT_SUCCESS);

    return EXIT_SUCCESS;
}

