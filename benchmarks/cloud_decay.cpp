// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Isolated retention-stage benchmark; run the command in docs/algorithm.md.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Clock = std::chrono::steady_clock;
void old_decay(const Cloud::Ptr& cloud, float now, float duration) {
    auto outliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    outliers->indices.resize(cloud->size());
    std::size_t n = 0, index = 0;
    for (const auto& point : cloud->points) {
        if (now - point.intensity > duration) outliers->indices[n++] = static_cast<int>(index);
        ++index;
    }
    outliers->indices.resize(n);
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(outliers);
    extract.setNegative(true);
    extract.filter(*cloud);
}
void new_decay(const Cloud::Ptr& cloud, float now, float duration) {
    const auto end = std::remove_if(cloud->points.begin(), cloud->points.end(),
        [now, duration](const Point& point) { return now - point.intensity > duration; });
    cloud->resize(static_cast<std::size_t>(end - cloud->points.begin()));
}
double percentile(std::vector<double> samples, double fraction) {
    std::sort(samples.begin(), samples.end());
    return samples[static_cast<std::size_t>(fraction * (samples.size() - 1))];
}
void run(std::size_t count, int iterations) {
    auto input = Cloud::Ptr(new Cloud);
    input->resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        input->points[i].x = static_cast<float>(i);
        input->points[i].intensity = static_cast<float>(i % 10);
    }
    std::vector<double> old_samples, new_samples;
    old_samples.reserve(iterations);
    new_samples.reserve(iterations);
    // Copy and output checks stay outside each timed retention pass.
    for (int i = 0; i < iterations; ++i) {
        auto old_cloud = Cloud::Ptr(new Cloud(*input));
        auto new_cloud = Cloud::Ptr(new Cloud(*input));
        double old_ms, new_ms;
        if (i % 2 == 0) {
            auto start = Clock::now();
            old_decay(old_cloud, 10.0f, 5.0f);
            auto middle = Clock::now();
            new_decay(new_cloud, 10.0f, 5.0f);
            auto end = Clock::now();
            old_ms = std::chrono::duration<double, std::milli>(middle - start).count();
            new_ms = std::chrono::duration<double, std::milli>(end - middle).count();
        } else {
            auto start = Clock::now();
            new_decay(new_cloud, 10.0f, 5.0f);
            auto middle = Clock::now();
            old_decay(old_cloud, 10.0f, 5.0f);
            auto end = Clock::now();
            new_ms = std::chrono::duration<double, std::milli>(middle - start).count();
            old_ms = std::chrono::duration<double, std::milli>(end - middle).count();
        }
        if (old_cloud->size() != new_cloud->size()) std::abort();
        for (std::size_t j = 0; j < old_cloud->size(); ++j)
            if (old_cloud->points[j].x != new_cloud->points[j].x ||
                old_cloud->points[j].intensity != new_cloud->points[j].intensity) std::abort();
        old_samples.push_back(old_ms);
        new_samples.push_back(new_ms);
    }
    std::cout << "points=" << count << " kept=" << count / 2
              << " old_median_ms=" << percentile(old_samples, 0.5)
              << " old_p95_ms=" << percentile(old_samples, 0.95)
              << " new_median_ms=" << percentile(new_samples, 0.5)
              << " new_p95_ms=" << percentile(new_samples, 0.95) << '\n';
}

}  // namespace

int main() {
    run(10000, 30);
    run(100000, 30);
    run(1000000, 12);
}
