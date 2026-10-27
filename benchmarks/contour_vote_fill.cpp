// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Isolated no-op contour-vote fill benchmark for an already recorded pair.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace {

std::size_t allocations = 0;

template <typename T>
struct CountingAllocator {
    using value_type = T;
    CountingAllocator() = default;
    template <typename U> CountingAllocator(const CountingAllocator<U>&) {}
    T* allocate(std::size_t count) {
        ++allocations;
        return std::allocator<T>{}.allocate(count);
    }
    void deallocate(T* data, std::size_t count) {
        std::allocator<T>{}.deallocate(data, count);
    }
};

template <typename T, typename U>
bool operator==(const CountingAllocator<T>&, const CountingAllocator<U>&) { return true; }
template <typename T, typename U>
bool operator!=(const CountingAllocator<T>&, const CountingAllocator<U>&) { return false; }

using History = std::deque<int, CountingAllocator<int>>;
using Votes = std::unordered_map<std::size_t, History>;
using Clock = std::chrono::steady_clock;

__attribute__((noinline)) void old_fill(Votes& first, Votes& second, int count) {
    const auto a = first.find(2);
    const auto b = second.find(1);
    History vote_a(count, 1), vote_b(count, 1);
    if (a == first.end() || b == second.end()) {
        first.insert({2, vote_a});
        second.insert({1, vote_b});
    }
}

__attribute__((noinline)) void new_fill(Votes& first, Votes& second, int count) {
    const auto a = first.find(2);
    const auto b = second.find(1);
    if (a == first.end() || b == second.end()) {
        History vote_a(count, 1), vote_b(count, 1);
        first.insert({2, vote_a});
        second.insert({1, vote_b});
    }
}

double percentile(std::vector<double> values, double fraction) {
    std::sort(values.begin(), values.end());
    return values[static_cast<std::size_t>(fraction * (values.size() - 1))];
}

}  // namespace

int main() {
    Votes first, second;
    first.emplace(2, History{1});
    second.emplace(1, History{1});
    constexpr int calls = 100000;
    std::vector<double> old_ms, new_ms;
    old_ms.reserve(20);
    new_ms.reserve(20);
    std::size_t old_allocations = 0, new_allocations = 0;
    for (int sample = 0; sample < 20; ++sample) {
        const auto measure = [&](bool old) {
            const auto before = allocations;
            const auto start = Clock::now();
            for (int i = 0; i < calls; ++i) {
                if (old) old_fill(first, second, 10);
                else new_fill(first, second, 10);
            }
            const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            (old ? old_allocations : new_allocations) = allocations - before;
            (old ? old_ms : new_ms).push_back(ms);
        };
        if (sample % 2 == 0) { measure(true); measure(false); }
        else { measure(false); measure(true); }
    }
    if (first.at(2) != History{1} || second.at(1) != History{1}) std::abort();
    std::cout << "calls=" << calls << " old_allocations=" << old_allocations
              << " new_allocations=" << new_allocations
              << " old_median_ms=" << percentile(old_ms, 0.5)
              << " old_p95_ms=" << percentile(old_ms, 0.95)
              << " new_median_ms=" << percentile(new_ms, 0.5)
              << " new_p95_ms=" << percentile(new_ms, 0.95) << '\n';
}
