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
Point coordinates;

/* Global command line knobs */
std::string cl_option_filename;
uint64_t cl_option_loop;

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

    //munlockall();

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
Usage: linux-robotic-arm-playback.app -f FILE -l 0          \n\
Used to playback the trajectory of a robotic arm.           \n\
                                                            \n\
    -f,--file=     Trajectory file to playback              \n\
    -l,--loops=    The number of loops to repeat the path   \n\
    -h,--help      Prints the usage and exit (this screen)  \n\
                                                            \n\
                                                            \n\
Example:                                                    \n\
linux-robotic-arm-playback.app -f trajectory.rec -l 0       \n\
");
    std::cerr << usage << std::endl;
    exit(EXIT_FAILURE);
}

void ProcessCLI(int argc, char *argv[])
{
    int c, option_index = 0;

    struct option long_options[] = {
        { "file"    , required_argument ,0, 'f'},
        { "loop "   , optional_argument ,0, 'l'},
        { "help"    , no_argument       ,0, 'h'},
        { 0         , 0                 ,0,  0 }
    };

    if (argc < 2)
        PrintUsage();

    while ((c = getopt_long(argc, argv, "f:l:h", long_options, &option_index)) != -1)
        switch(c) {

            case 'f':
                cl_option_filename.assign(optarg);
                break;

            case 'l':
                cl_option_loop = (uint64_t)atol(optarg);
                break;

            case 'h':
            case '?':
            default:
                PrintUsage();

        }
}

void ParseTrajectoryFile(const std::string &file)
{
    logger << "I: Loading trajectory file: \"" << file << "\"" << std::endl;

    std::vector<Point> points;

    std::string line;
    std::ifstream infile(file);

    if(infile.is_open()) {

        while(infile.good() ) {

            if (std::getline(infile, line)) {

                /* Store the line being parsed as string for manipulation */
                char *token = std::strtok((char *)line.c_str(), " ");
                std::vector<std::string> split_line;

                /* Splitting elements and saving them into a vector */
                while(token != NULL) {
                    split_line.push_back(token);
                    token = std::strtok(NULL, " ");
                }

                /* First 3 fields form the coordinates */
                Point p;
                p.x = std::stod(split_line[0]);
                p.y = std::stod(split_line[1]);
                p.z = std::stod(split_line[2]);
                points.push_back(p);

            }

        }
        
        infile.close();
        
    } else {
        logger << "E: Failed to load the trajectory file" << std::endl;
        return;
    }

    logger << "I: Loaded " << points.size() << " points from the trajectory file" << std::endl;
}

int main(int argc, char *argv[])
{
    /* Process the trajectory filename and arguments */
    ProcessCLI(argc, argv);

    //mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Preload the input file, and start loading it in memory */
    ParseTrajectoryFile(cl_option_filename);

    exit(-99);

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

