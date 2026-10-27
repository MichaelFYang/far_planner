// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#ifndef FAR_PLANNER_GOAL_GRID_SEARCH_H
#define FAR_PLANNER_GOAL_GRID_SEARCH_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "far_planner/grid.h"

namespace far_planner {

// Reuse scratch storage across goal updates; clear only cells reached by the search.
class GoalGridSearch {
public:
    // Return the first free cell in the existing left/up/right/down BFS order.
    int Find(const grid_ns::Grid<char>& grid, int seed, char free_value) {
        if (!grid.InRange(seed)) return -1;
        if (visited_.size() != static_cast<std::size_t>(grid.GetCellNumber())) {
            visited_.assign(grid.GetCellNumber(), 0);
        }
        queue_.clear();
        queue_.push_back(seed);
        visited_[seed] = 1;

        constexpr std::array<int, 4> dx = {-1, 0, 1, 0};
        constexpr std::array<int, 4> dy = {0, 1, 0, -1};
        int result = -1;
        for (std::size_t head = 0; head < queue_.size(); ++head) {
            const int current = queue_[head];
            if (grid.GetCellValue(current) == free_value) {
                result = current;
                break;
            }
            const Eigen::Vector3i sub = grid.Ind2Sub(current);
            for (std::size_t i = 0; i < dx.size(); ++i) {
                const Eigen::Vector3i neighbor(sub.x() + dx[i], sub.y() + dy[i], 0);
                if (!grid.InRange(neighbor)) continue;
                const int index = grid.Sub2Ind(neighbor);
                if (visited_[index]) continue;
                queue_.push_back(index);
                visited_[index] = 1;
            }
        }
        for (const int index : queue_) visited_[index] = 0;
        return result;
    }

private:
    std::vector<std::uint8_t> visited_;
    std::vector<int> queue_;
};

}  // namespace far_planner

#endif
