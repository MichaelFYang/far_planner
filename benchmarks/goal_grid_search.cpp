// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Isolated goal-grid BFS benchmark; polygon checks and map updates are excluded.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "far_planner/goal_grid_search.h"

namespace {
using Clock = std::chrono::steady_clock;

int PreviousSearch(const grid_ns::Grid<char>& grid, int seed) {
    if (!grid.InRange(seed)) return -1;
    const std::array<int, 4> dx = {-1, 0, 1, 0};
    const std::array<int, 4> dy = {0, 1, 0, -1};
    std::deque<int> queue;
    std::unordered_set<int> visited;
    queue.push_back(seed);
    visited.insert(seed);
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop_front();
        if (grid.GetCellValue(current) == 2) return current;
        for (std::size_t i = 0; i < dx.size(); ++i) {
            Eigen::Vector3i sub = grid.Ind2Sub(current);
            sub.x() += dx[i];
            sub.y() += dy[i];
            sub.z() = 0;
            if (!grid.InRange(sub)) continue;
            const int index = grid.Sub2Ind(sub);
            if (visited.insert(index).second) queue.push_back(index);
        }
    }
    return -1;
}

double Percentile(std::vector<double> times, std::size_t index) {
    std::sort(times.begin(), times.end());
    return times[index];
}

void Measure(int width, int free_distance, int calls) {
    grid_ns::Grid<char> grid(Eigen::Vector3i(width, width, 1), 0);
    const int center = width / 2;
    const int seed = grid.Sub2Ind(center, center, 0);
    if (free_distance >= 0) grid.GetCell(center + free_distance, center, 0) = 2;
    const int expected = PreviousSearch(grid, seed);
    far_planner::GoalGridSearch search;
    if (expected != search.Find(grid, seed, 2)) std::exit(1);

    std::vector<double> before, after;
    volatile int sink = 0;
    const auto run = [&](bool updated) {
        const auto start = Clock::now();
        for (int i = 0; i < calls; ++i) {
            sink = updated ? search.Find(grid, seed, 2)
                           : PreviousSearch(grid, seed);
        }
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        (updated ? after : before).push_back(ms);
    };
    for (int sample = 0; sample < 20; ++sample) {
        if (sample % 2) { run(true); run(false); }
        else { run(false); run(true); }
    }
    std::cout << "width=" << width << " free_distance=" << free_distance
              << " calls=" << calls << " result=" << expected
              << " old_median_ms=" << Percentile(before, 10)
              << " old_p95_ms=" << Percentile(before, 18)
              << " new_median_ms=" << Percentile(after, 10)
              << " new_p95_ms=" << Percentile(after, 18)
              << " sink=" << sink << '\n';
}
}  // namespace

int main() {
    Measure(101, 1, 200);
    Measure(101, 20, 20);
    Measure(301, 1, 200);
    Measure(301, 80, 3);
    Measure(301, -1, 3);
}
