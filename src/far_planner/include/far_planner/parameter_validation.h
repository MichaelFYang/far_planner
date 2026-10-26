// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef FAR_PLANNER_PARAMETER_VALIDATION_H
#define FAR_PLANNER_PARAMETER_VALIDATION_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace far_planner {

inline void RequirePositiveFinite(double value, const std::string& name) {
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(name + " must be positive and finite");
}

inline void RequireNonNegativeFinite(double value, const std::string& name) {
    if (!std::isfinite(value) || value < 0.0)
        throw std::invalid_argument(name + " must be nonnegative and finite");
}

inline void RequirePositiveInteger(int value, const std::string& name) {
    if (value <= 0) throw std::invalid_argument(name + " must be a positive integer");
}

inline void RequireNonNegativeInteger(int value, const std::string& name) {
    if (value < 0) throw std::invalid_argument(name + " must be a nonnegative integer");
}

inline std::uint16_t CheckedRobotId(int value) {
    if (value < 0 || value > std::numeric_limits<std::uint16_t>::max())
        throw std::invalid_argument("graph_msger/robot_id must be between 0 and 65535");
    return static_cast<std::uint16_t>(value);
}

inline int CheckedDimension(double span, double resolution, const std::string& name,
                            int extra = 0, bool odd = false) {
    RequirePositiveFinite(span, name + " span");
    RequirePositiveFinite(resolution, name + " resolution");
    const double cells = std::ceil(span / resolution);
    if (extra < 0 || extra >= std::numeric_limits<int>::max() || !std::isfinite(cells) ||
        cells > static_cast<double>(std::numeric_limits<int>::max() - extra - (odd ? 1 : 0)))
        throw std::invalid_argument(name + " exceeds the grid index range");
    int count = static_cast<int>(cells) + extra;
    if (odd && count % 2 == 0) ++count;
    return count;
}

inline int CheckedRoundedDimension(double value, const std::string& name) {
    RequirePositiveFinite(value, name);
    const double rounded = std::round(value);
    if (!std::isfinite(rounded) || rounded < 1.0 ||
        rounded > static_cast<double>(std::numeric_limits<int>::max()))
        throw std::invalid_argument(name + " exceeds the supported image size");
    return static_cast<int>(rounded);
}

inline std::int64_t CheckedTimerPeriodNs(double frequency) {
    RequirePositiveFinite(frequency, "main_run_freq");
    const double period_ns = 1e9 / frequency;
    // ROS 1 Duration stores seconds in a signed 32-bit field.
    const double max_ns = static_cast<double>(std::numeric_limits<std::int32_t>::max()) * 1e9;
    if (!std::isfinite(period_ns) || period_ns < 1.0 || period_ns > max_ns)
        throw std::invalid_argument("main_run_freq produces an unsupported timer period");
    return static_cast<std::int64_t>(std::round(period_ns));
}

}  // namespace far_planner
#endif
