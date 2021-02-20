#include "libbsn/time/TimeData.hpp"

using namespace std;

namespace bsn {
    namespace time {

        TimeData::TimeData() {}

        TimeData::~TimeData() {}

        timespec TimeData::elapsedTime(timespec &now, timespec &ref) {

            timespec diff;

            if ((now.tv_nsec - ref.tv_nsec) < 0) {
                diff.tv_sec = now.tv_sec - ref.tv_sec - 1;
                diff.tv_nsec = now.tv_nsec - ref.tv_nsec + 1000000000L;
            }
            else {
                diff.tv_sec = now.tv_sec - ref.tv_sec;
                diff.tv_nsec = now.tv_nsec - ref.tv_nsec;
            }

            return diff;
        }

        string TimeData::get_time(){
            char buffer[26];
            int32_t millisec;
            struct tm* tm_info;
            struct timeval tv;

            gettimeofday(&tv, NULL);

            millisec = lrint(tv.tv_usec/1000.0); // Round to nearest millisec
            if (millisec>=1000) { // Allow for rounding up to nearest second
            millisec -=1000;
            tv.tv_sec++;
            }

            tm_info = localtime(&tv.tv_sec);

            strftime(buffer, 26, "%Y:%m:%d %H:%M:%S", tm_info);

            string data = buffer;
            data += ':' + to_string(millisec);

            return data;
        }

    }
}