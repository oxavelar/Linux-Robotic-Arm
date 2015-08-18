#pragma once
#include <iostream>
#include <chrono>
#include <ctime>
#include <string>


#define appname "Linux-Robotic-Arm"
#define logger std::cout << appname << ": " << timestamp().c_str() << ": "

inline const std::string timestamp(void)
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

