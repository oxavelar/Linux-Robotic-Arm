#pragma once
#include <iostream>
#include <chrono>
#include <ctime>
#include <string>
#include <streambuf>
#include <ncurses.h>
#include <fcntl.h>

#define appname "Robotic-Arm"
#define logger std::cout << "[" << toolbox::timestamp().c_str() << "] " appname ": "

namespace toolbox
{
    inline double get_cpu_load(void)
    {
       int file_handler;
       char file_buffer[512];
       double load;
       
       file_handler = open("/proc/loadavg", O_RDONLY);
       if(file_handler > 0) {
           read(file_handler, file_buffer, sizeof(file_buffer) - 1);
           sscanf(file_buffer, "%lf", &load);
           close(file_handler);
           load = 100.0 * load;
       } else {
           load = -EINVAL;
       }
       return load;
    }

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
    
    class ncursesbuf: public std::streambuf {
        public:
            explicit ncursesbuf() {}
            virtual int overflow(int c)
            {
                printw("%c", c);
                return 0;
            }
    };

    class ncurses_stream : public std::ostream {
        public:
            explicit ncurses_stream(std::ostream &o) : 
            src(o), srcbuf(o.rdbuf()), std::ostream(&tbuf)
            {
                o.rdbuf(rdbuf());
                o.flush();
            }

            ~ncurses_stream() {
                src.rdbuf(srcbuf);
            }
        private:
            std::ostream &src;
            std::streambuf * const srcbuf;
            ncursesbuf tbuf;
    };
}

