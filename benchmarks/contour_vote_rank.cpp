// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Compare the prior sorted contour-vote ranking with the production selector.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <numeric>
#include <random>
#include <unordered_map>
#include <vector>

#include "far_planner/vote_selection.h"

using Votes = std::unordered_map<std::size_t, std::deque<int>>;
using Clock = std::chrono::steady_clock;

bool qualifies(const std::deque<int>& history) {
    const int sum = std::accumulate(history.begin(), history.end(), 0);
    return static_cast<float>(sum) > std::floor(history.size() / 3.0f);
}

std::size_t old_select(const Votes& votes, std::vector<std::size_t>* selected = nullptr) {
    std::vector<int> ranked;
    for (const auto& item : votes) {
        if (qualifies(item.second))
            ranked.push_back(std::accumulate(item.second.begin(), item.second.end(), 0));
    }
    std::sort(ranked.begin(), ranked.end(), std::greater<int>());
    std::size_t checksum = 0;
    for (const auto& item : votes) {
        const int score = std::accumulate(item.second.begin(), item.second.end(), 0);
        int rank = 0;
        while (rank < static_cast<int>(ranked.size()) && score < ranked[rank]) ++rank;
        if (rank < 2 && qualifies(item.second)) {
            checksum += item.first + 1;
            if (selected) selected->push_back(item.first);
        }
    }
    return checksum;
}

std::size_t new_select(const Votes& votes, std::vector<std::size_t>* selected = nullptr) {
    far_planner::TopTwoScores top_scores;
    for (const auto& item : votes) {
        if (!qualifies(item.second)) continue;
        const int score = std::accumulate(item.second.begin(), item.second.end(), 0);
        top_scores.Observe(score);
    }
    std::size_t checksum = 0;
    for (const auto& item : votes) {
        const int score = std::accumulate(item.second.begin(), item.second.end(), 0);
        if (qualifies(item.second) && top_scores.Includes(score)) {
            checksum += item.first + 1;
            if (selected) selected->push_back(item.first);
        }
    }
    return checksum;
}

double percentile(std::vector<double> values, double fraction) {
    std::sort(values.begin(), values.end());
    return values[static_cast<std::size_t>(fraction * (values.size() - 1))];
}

void run(std::size_t entries, int repetitions) {
    std::mt19937 random(12345 + static_cast<unsigned>(entries));
    std::bernoulli_distribution vote(0.48);
    std::vector<Votes> inputs(32);
    for (auto& input : inputs) {
        for (std::size_t id = 0; id < entries; ++id) {
            auto& history = input[id];
            for (int j = 0; j < 10; ++j) history.push_back(vote(random) ? 1 : 0);
        }
        std::vector<std::size_t> old_ids, new_ids;
        old_select(input, &old_ids);
        new_select(input, &new_ids);
        std::sort(old_ids.begin(), old_ids.end());
        std::sort(new_ids.begin(), new_ids.end());
        if (old_ids != new_ids) std::abort();
    }
    std::vector<double> old_ms, new_ms;
    volatile std::size_t checksum = 0;
    for (int sample = 0; sample < 20; ++sample) {
        const auto measure = [&](bool old) {
            const auto start = Clock::now();
            for (int i = 0; i < repetitions; ++i)
                checksum += old ? old_select(inputs[(i + sample) % inputs.size()])
                                : new_select(inputs[(i + sample) % inputs.size()]);
            return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        };
        if (sample % 2 == 0) {
            old_ms.push_back(measure(true));
            new_ms.push_back(measure(false));
        } else {
            new_ms.push_back(measure(false));
            old_ms.push_back(measure(true));
        }
    }
    std::cout << "entries=" << entries << " checks=" << checksum
              << " old_median_ms=" << percentile(old_ms, 0.5)
              << " old_p95_ms=" << percentile(old_ms, 0.95)
              << " new_median_ms=" << percentile(new_ms, 0.5)
              << " new_p95_ms=" << percentile(new_ms, 0.95) << '\n';
}

int main() {
    run(4, 20000);
    run(16, 10000);
    run(64, 3000);
    run(256, 1000);
}
