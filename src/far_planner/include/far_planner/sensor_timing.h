// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef FAR_PLANNER_SENSOR_TIMING_H
#define FAR_PLANNER_SENSOR_TIMING_H

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <utility>

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

// Hold measurements while a matching stamped TF or odometry pose arrives.
// Entries are ordered by stamp, bounded in count, and expire by steady time.
template <typename Message>
class PendingMeasurementQueue {
public:
    explicit PendingMeasurementQueue(std::size_t capacity = 32)
        : capacity_(std::max<std::size_t>(1, capacity)) {}

    struct Entry {
        std::uint64_t stamp;
        Message message;
        std::uint64_t arrived_ns;
    };

    bool Push(std::uint64_t stamp, Message message, std::uint64_t last_accepted,
              std::uint64_t arrived_ns) {
        if (stamp == 0 || stamp <= last_accepted ||
            (!entries_.empty() && stamp <= entries_.back().stamp)) return false;
        entries_.push_back({stamp, std::move(message), arrived_ns});
        if (entries_.size() > capacity_) entries_.pop_front();
        return true;
    }

    std::size_t DiscardExpired(std::uint64_t now_ns, std::uint64_t max_age_ns) {
        std::size_t removed = 0;
        while (!entries_.empty() && now_ns > entries_.front().arrived_ns &&
               now_ns - entries_.front().arrived_ns > max_age_ns) {
            entries_.pop_front();
            ++removed;
        }
        return removed;
    }

    bool HasReady(std::uint64_t latest_odometry) const {
        return !entries_.empty() && entries_.front().stamp <= latest_odometry;
    }

    bool HasStampWithin(std::uint64_t stamp, std::uint64_t tolerance) const {
        return std::any_of(entries_.begin(), entries_.end(), [stamp, tolerance](const Entry& entry) {
            return (entry.stamp > stamp ? entry.stamp - stamp : stamp - entry.stamp) <= tolerance;
        });
    }

    const Entry& Front() const { return entries_.front(); }

    Entry PopReady() {
        Entry entry = std::move(entries_.front());
        entries_.pop_front();
        return entry;
    }

    std::size_t Size() const { return entries_.size(); }
    void Clear() { entries_.clear(); }

private:
    std::size_t capacity_;
    std::deque<Entry> entries_;
};

// Keep processed measurements available for later, timestamped consumers.
// The closest sample wins; an exact or earlier sample wins a tie.
template <typename Value>
class RecentMeasurementHistory {
public:
    explicit RecentMeasurementHistory(std::size_t capacity = 32)
        : capacity_(std::max<std::size_t>(1, capacity)) {}

    struct Entry {
        std::uint64_t stamp;
        Value value;
    };

    bool Add(std::uint64_t stamp, Value value) {
        if (stamp == 0 || (!entries_.empty() && stamp <= entries_.back().stamp)) return false;
        entries_.push_back({stamp, std::move(value)});
        if (entries_.size() > capacity_) entries_.pop_front();
        return true;
    }

    const Entry* Nearest(std::uint64_t stamp, std::uint64_t tolerance) const {
        const auto after = std::lower_bound(entries_.begin(), entries_.end(), stamp,
            [](const Entry& entry, std::uint64_t time) { return entry.stamp < time; });
        const Entry* best = nullptr;
        if (after != entries_.end() && after->stamp - stamp <= tolerance) best = &*after;
        if (after != entries_.begin()) {
            const auto before = std::prev(after);
            if (stamp - before->stamp <= tolerance &&
                (best == nullptr || stamp - before->stamp <= best->stamp - stamp)) {
                best = &*before;
            }
        }
        return best;
    }

    void PruneBefore(std::uint64_t min_stamp) {
        while (!entries_.empty() && entries_.front().stamp < min_stamp) entries_.pop_front();
    }

    std::size_t Size() const { return entries_.size(); }
    void Clear() { entries_.clear(); }

private:
    std::size_t capacity_;
    std::deque<Entry> entries_;
};

inline std::uint64_t SteadyNowNs() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

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

// Reject a cloud already old when it arrives. Once queued, later odometry
// must not invalidate its measurement-time pose while stamped TF is pending.
inline bool SensorStampFreshOnArrival(std::uint64_t stamp, std::uint64_t latest_odometry,
                                     std::uint64_t tolerance) {
    return stamp != 0 && (latest_odometry == 0 || stamp >= latest_odometry ||
                          WithinSensorSkew(stamp, latest_odometry, tolerance));
}

}  // namespace far_planner
#endif
