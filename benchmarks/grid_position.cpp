// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// In-range conversion benchmark; see docs/algorithm.md for limitations.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <Eigen/Core>

#include "far_planner/grid.h"

namespace {

using Clock = std::chrono::steady_clock;

Eigen::Vector3i LegacyPositionToSub(const Eigen::Vector3d& pos,
                                    const Eigen::Vector3d& origin,
                                    const Eigen::Vector3d& inverse) {
    Eigen::Vector3i sub;
    for (int axis = 0; axis < 3; ++axis) {
        const double offset = pos(axis) - origin(axis);
        sub(axis) = offset > -1e-7 ? static_cast<int>(offset * inverse(axis)) : -1;
    }
    return sub;
}

template <typename Convert>
std::uint64_t Run(const std::vector<Eigen::Vector3d>& points, Convert convert) {
    std::uint64_t sum = 0;
    for (int repeat = 0; repeat < 50; ++repeat) {
        for (const auto& point : points) {
            const auto sub = convert(point);
            sum += static_cast<std::uint64_t>(sub.x() * 3 + sub.y() * 5 + sub.z());
        }
    }
    return sum;
}

double Median(std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

}  // namespace

int main() {
    grid_ns::Grid<int, true> grid(Eigen::Vector3i(1024, 1024, 1), 0);
    std::vector<Eigen::Vector3d> points;
    points.reserve(100000);
    for (int index = 0; index < 100000; ++index) {
        points.emplace_back((index * 37 % 1000) + 0.25,
                            (index * 71 % 1000) + 0.5, 0.25);
    }
    const auto origin = grid.GetOrigin();
    const auto inverse = grid.GetResolutionInv();
    const auto old_convert = [&](const Eigen::Vector3d& point) {
        return LegacyPositionToSub(point, origin, inverse);
    };
    const auto new_convert = [&](const Eigen::Vector3d& point) {
        return grid.Pos2Sub(point);
    };
    if (Run(points, old_convert) != Run(points, new_convert)) std::abort();

    std::vector<double> old_ms, new_ms;
    for (int iteration = 0; iteration < 10; ++iteration) {
        const auto first = Clock::now();
        const auto first_sum = iteration % 2 ? Run(points, new_convert) : Run(points, old_convert);
        const auto middle = Clock::now();
        const auto second_sum = iteration % 2 ? Run(points, old_convert) : Run(points, new_convert);
        const auto last = Clock::now();
        if (first_sum != second_sum) std::abort();
        const double first_ms = std::chrono::duration<double, std::milli>(middle - first).count();
        const double second_ms = std::chrono::duration<double, std::milli>(last - middle).count();
        old_ms.push_back(iteration % 2 ? second_ms : first_ms);
        new_ms.push_back(iteration % 2 ? first_ms : second_ms);
    }
    std::cout << "positions=5000000 old_median_ms=" << Median(old_ms)
              << " checked_median_ms=" << Median(new_ms) << '\n';
}
