#ifndef TIMEDATA_HPP
#define TIMEDATA_HPP

#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <string>
#include <iostream>
#include <unistd.h>

namespace bsn {
    namespace time {

        class TimeData {
            public:
                TimeData();
                ~TimeData();

                TimeData(const TimeData & /*obj*/);
                TimeData &operator=(const TimeData & /*obj*/);

                timespec elapsedTime(timespec &, timespec &);
                std::string get_time();

        };

    }
}

#endif