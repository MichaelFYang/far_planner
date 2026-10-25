// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef FAR_PLANNER_SENSOR_TIMING_H
#define FAR_PLANNER_SENSOR_TIMING_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>

namespace far_planner {

// ROS-independent measurement-time policies. Stamps are original ROS nanoseconds.
class OdometryHistory {
public:
    using Position = std::array<double, 3>;

    bool Add(std::uint64_t stamp, const Position& position) {
        if (stamp == 0 || stamp <= LatestStamp() ||
            !std::all_of(position.begin(), position.end(),
                         [](double value) { return std::isfinite(value); })) return false;
        samples_.push_back({stamp, position});
        // Bound both storage and history, independently of publisher frequency.
        while (samples_.size() > 2000 || stamp - samples_.front().stamp > 10000000000ULL) {
            samples_.pop_front();
        }
        return true;
    }

    bool PositionAt(std::uint64_t stamp, std::uint64_t tolerance, Position& output) const {
        if (stamp == 0 || samples_.empty()) return false;
        const auto after = std::lower_bound(samples_.begin(), samples_.end(), stamp,
            [](const Sample& sample, std::uint64_t time) { return sample.stamp < time; });
        if (after == samples_.end()) return false; // Never extrapolate.
        if (after->stamp == stamp) {
            output = after->position;
            return true;
        }
        if (after == samples_.begin()) return false;
        const auto before = std::prev(after);
        if (stamp - before->stamp > tolerance || after->stamp - stamp > tolerance) return false;
        const double ratio = static_cast<double>(stamp - before->stamp) /
                             static_cast<double>(after->stamp - before->stamp);
        for (std::size_t i = 0; i < output.size(); ++i) {
            output[i] = before->position[i] + ratio * (after->position[i] - before->position[i]);
        }
        return true;
    }

    std::uint64_t LatestStamp() const { return samples_.empty() ? 0 : samples_.back().stamp; }
    void Clear() { samples_.clear(); }

private:
    struct Sample {
        std::uint64_t stamp;
        Position position;
    };
    std::deque<Sample> samples_;
};

// Called from the serialized planner callbacks, not from the TF listener thread.
class ClockEpoch {
public:
    bool Observe(std::int64_t now) {
        const bool rewound = initialized_ && now < last_;
        last_ = now;
        initialized_ = true;
        return rewound;
    }
private:
    std::int64_t last_ = 0;
    bool initialized_ = false;
};

inline bool WithinSensorSkew(std::uint64_t first, std::uint64_t second,
                             std::uint64_t tolerance) {
    if (first == 0 || second == 0) return false;
    return (first > second ? first - second : second - first) <= tolerance;
}

}  // namespace far_planner
#endif
