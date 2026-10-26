// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Isolated contour collection compaction; OpenCV geometry is excluded.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

namespace {
using Contour = std::vector<std::array<int, 2>>;
using Contours = std::vector<Contour>;
using Clock = std::chrono::steady_clock;

void PreviousCompact(Contours& contours, const std::vector<bool>& remove) {
    const Contours copy = contours;
    contours.clear();
    for (std::size_t i = 0; i < copy.size(); ++i) {
        if (!remove[i]) contours.push_back(copy[i]);
    }
}

void CompactInPlace(Contours& contours, const std::vector<bool>& remove) {
    std::size_t kept = 0;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        if (remove[i]) continue;
        if (kept != i) contours[kept] = std::move(contours[i]);
        ++kept;
    }
    contours.resize(kept);
}

double Percentile(std::vector<double> samples, std::size_t index) {
    std::sort(samples.begin(), samples.end());
    return samples[index];
}
}  // namespace

int main() {
    Contours source(512);
    std::vector<bool> remove(source.size());
    for (std::size_t i = 0; i < source.size(); ++i) {
        remove[i] = i % 3 == 0;
        for (int vertex = 0; vertex < 256; ++vertex) {
            source[i].push_back({static_cast<int>(i), vertex});
        }
    }
    Contours expected = source;
    PreviousCompact(expected, remove);
    std::vector<double> before, after;
    for (int sample = 0; sample < 40; ++sample) {
        const auto run = [&](bool updated) {
            Contours data = source;  // Excluded from timing.
            const auto start = Clock::now();
            if (updated) CompactInPlace(data, remove);
            else PreviousCompact(data, remove);
            const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            if (data != expected) std::exit(1);
            (updated ? after : before).push_back(ms);
        };
        if (sample % 2) { run(true); run(false); }
        else { run(false); run(true); }
    }
    std::cout << "contours=512 vertices_per_contour=256 removed=171"
              << " old_median_ms=" << Percentile(before, 20)
              << " old_p95_ms=" << Percentile(before, 38)
              << " new_median_ms=" << Percentile(after, 20)
              << " new_p95_ms=" << Percentile(after, 38) << '\n';
}
