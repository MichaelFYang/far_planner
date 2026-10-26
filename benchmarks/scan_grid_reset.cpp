// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Isolated dense versus touched-cell reset probe; write tracking is excluded.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

using Clock = std::chrono::steady_clock;

int main() {
    constexpr std::size_t cells = 200 * 200 * 33;  // Default 15 m / 0.15 m scan grid.
    std::vector<unsigned char> grid(cells, 0);
    std::vector<int> indices(cells);
    std::iota(indices.begin(), indices.end(), 0);
    std::mt19937 rng(1234);
    std::shuffle(indices.begin(), indices.end(), rng);

    for (std::size_t count : {1000U, 20000U, 100000U, 500000U}) {
        for (bool random_order : {false, true}) {
            std::vector<int> touched(indices.begin(), indices.begin() + count);
            if (!random_order) std::sort(touched.begin(), touched.end());
            std::vector<double> dense, sparse;
            for (int sample = 0; sample < 40; ++sample) {
                const auto measure = [&](bool sparse_reset) {
                    for (const int index : touched) grid[index] = 1;
                    const auto start = Clock::now();
                    if (sparse_reset) {
                        for (const int index : touched) grid[index] = 0;
                    } else {
                        std::fill(grid.begin(), grid.end(), 0);
                    }
                    const double us = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
                    (sparse_reset ? sparse : dense).push_back(us);
                };
                if (sample % 2) { measure(true); measure(false); }
                else { measure(false); measure(true); }
            }
            std::sort(dense.begin(), dense.end());
            std::sort(sparse.begin(), sparse.end());
            std::cout << "cells=" << cells << " touched=" << count
                      << " random_order=" << random_order << " dense_median_us=" << dense[20]
                      << " sparse_median_us=" << sparse[20] << '\n';
        }
    }
}
