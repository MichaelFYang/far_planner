// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Isolated square inflation comparison; grid reset and point conversion are excluded.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

#include "far_planner/grid.h"

using Clock = std::chrono::steady_clock;
using Grid = grid_ns::Grid<char>;

void PaintOld(Grid& grid, const std::vector<Eigen::Vector3i>& centers, int radius) {
    for (const auto& center : centers) {
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                const Eigen::Vector3i cell(center.x() + dx, center.y() + dy, 0);
                if (grid.InRange(cell)) grid.GetCell(grid.Sub2Ind(cell)) |= 1;
            }
        }
    }
}

void PaintClipped(Grid& grid, const std::vector<Eigen::Vector3i>& centers, int radius) {
    for (const auto& center : centers) {
        grid.ForEachXYCellInRadius(center, radius,
            [&](int index) { grid.GetCell(index) |= 1; });
    }
}

int main() {
    Grid old_grid(Eigen::Vector3i(101, 101, 1), 0);
    Grid clipped_grid(Eigen::Vector3i(101, 101, 1), 0);
    std::mt19937 rng(2026);
    std::vector<Eigen::Vector3i> centers;
    centers.reserve(50000);
    for (int reach : {350, 70, 50}) {
        std::uniform_int_distribution<int> coordinate(50 - reach, 50 + reach);
        centers.clear();
        for (int i = 0; i < 50000; ++i) {
            centers.emplace_back(coordinate(rng), coordinate(rng), 0);
        }
        for (int radius : {1, 8}) {
            old_grid.ReInitGrid(0);
            clipped_grid.ReInitGrid(0);
            PaintOld(old_grid, centers, radius);
            PaintClipped(clipped_grid, centers, radius);
            for (int index = 0; index < old_grid.GetCellNumber(); ++index) {
                if (old_grid.GetCellValue(index) != clipped_grid.GetCellValue(index)) {
                    throw std::runtime_error("Inflated grid contents differ");
                }
            }

            std::vector<double> old_times, clipped_times;
            for (int sample = 0; sample < 20; ++sample) {
                const auto measure = [&](bool clipped) {
                    auto& grid = clipped ? clipped_grid : old_grid;
                    grid.ReInitGrid(0);
                    const auto start = Clock::now();
                    if (clipped) PaintClipped(grid, centers, radius);
                    else PaintOld(grid, centers, radius);
                    const double milliseconds =
                        std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                    (clipped ? clipped_times : old_times).push_back(milliseconds);
                };
                if (sample % 2 == 0) { measure(false); measure(true); }
                else { measure(true); measure(false); }
            }
            std::sort(old_times.begin(), old_times.end());
            std::sort(clipped_times.begin(), clipped_times.end());
            std::cout << "centers=" << centers.size() << " grid=101x101 reach=" << reach
                      << " radius=" << radius << " old_median_ms=" << old_times[10]
                      << " clipped_median_ms=" << clipped_times[10] << '\n';
        }
    }
}
