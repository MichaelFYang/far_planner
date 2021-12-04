#ifndef TIME_MEASURE_H
#define TIME_MEASURE_H

#include <chrono>
#include <iostream>

using namespace std;
using namespace chrono;
typedef high_resolution_clock Clock;

class TimeMeasure {
private:
    time_point<Clock> start_time_;
    time_point<Clock> end_time_;
    bool is_clock_start = false;
public:
    TimeMeasure() = default;
    ~TimeMeasure() = default;

    inline void start_time() {
        if (is_clock_start) {
            cout<<"Timer has already start."<<endl;
        }
        start_time_ = Clock::now();
        is_clock_start = true;
    }

    inline void end_time(string timer_name) {
        if (is_clock_start) {
            end_time_ = Clock::now();
            const auto duration = duration_cast<microseconds>(end_time_ - start_time_);
            cout<<timer_name<<" "<<"Time: "<<duration.count() / 1000.0<<"m"<<endl;
        } else {
            cout<<"Timer has not start yet."<<endl;
        }
        is_clock_start = false;
    }


};

#endif