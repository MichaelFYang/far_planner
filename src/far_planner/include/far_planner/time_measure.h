#ifndef TIME_MEASURE_H
#define TIME_MEASURE_H

#include <chrono>
#include <unordered_map>
#include <iostream>

using namespace std;
using namespace chrono;
typedef high_resolution_clock Clock;

typedef unordered_map<string, time_point<Clock>> TimerInstant;

class TimeMeasure {
private:
    TimerInstant timer_stack_;
public:
    TimeMeasure() = default;
    ~TimeMeasure() = default;

    inline void start_time(const string& timer_name, const bool& is_reset=false) {
        const auto it = timer_stack_.find(timer_name);
        const auto start_time = Clock::now();
        if (it == timer_stack_.end()) {
            timer_stack_.insert({timer_name, start_time});
        } else if (is_reset) {
            it->second = start_time;
        }
    }

    inline double end_time(const string& timer_name, const bool& is_output=true) {
        const auto it = timer_stack_.find(timer_name);
        if (it != timer_stack_.end()) {
            const auto end_time = Clock::now();
            const auto duration = duration_cast<microseconds>(end_time - it->second);
            const double time_duration = duration.count() / 1000.0;
            if (is_output) std::cout<<"    "<<timer_name<<" "<<"Time: "<<time_duration<<"ms"<<std::endl;
            timer_stack_.erase(it);
            return time_duration;
        } 
        return -1.0;
    }

    inline double record_time(const string& timer_name) {
        const auto it = timer_stack_.find(timer_name);
        if (it != timer_stack_.end()) {
            const auto cur_time = Clock::now();
            const auto duration = duration_cast<microseconds>(cur_time - it->second);
            const double time_duration = duration.count() / 1000.0;
            return time_duration;
        }
        return -1.0;
    }
};

#endif
