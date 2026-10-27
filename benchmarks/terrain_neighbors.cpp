// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
// Time the actual terrain-neighbor filter; prepare inputs and verify sets untimed.

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <unordered_set>

#include "far_planner/map_handler.h"

class MapHandlerTestAccess {
public:
    static std::unordered_set<int> Candidates() { return MapHandler::neighbor_obs_indices_; }
    static const grid_ns::Grid<PointCloudPtr, true>& World() {
        return *MapHandler::world_obs_cloud_grid_;
    }
    static void Filter(MapHandler& map, std::unordered_set<int>& cells,
                       std::unordered_set<int>& extended) {
        map.ObsNeighborCloudWithTerrain(cells, extended);
    }
};

// Linear XY reference uses the emitted terrain samples, without the KD-tree
// or the production filter. It also checks the one-cell downward extension.
void Reference(const std::unordered_set<int>& candidates, const PointCloud& heights,
               float radius, float cell_height, std::unordered_set<int>& kept,
               std::unordered_set<int>& extended)
{
    const auto& grid = MapHandlerTestAccess::World();
    for (int index : candidates) {
        const auto position = grid.Ind2Pos(index);
        float low = std::numeric_limits<float>::infinity();
        float high = -std::numeric_limits<float>::infinity();
        for (const auto& sample : heights) {
            const float dx = sample.x - static_cast<float>(position.x());
            const float dy = sample.y - static_cast<float>(position.y());
            if (dx * dx + dy * dy >= radius * radius) continue;
            low = std::min(low, sample.z);
            high = std::max(high, sample.z);
        }
        if (position.z() + cell_height <= low ||
            position.z() - cell_height >= high + FARUtil::kTolerZ) continue;
        kept.insert(index);
        auto sub = grid.Ind2Sub(index);
        extended.insert(index);
        --sub.z();
        if (grid.InRange(sub)) extended.insert(grid.Sub2Ind(sub));
    }
}

int main(int argc, char** argv)
{
    const int calls = argc > 1 ? std::atoi(argv[1]) : 100;
    if (calls <= 0) return 1;
    FARUtil::kLeafSize = 0.15f;
    FARUtil::robot_dim = 0.8f;
    FARUtil::vehicle_height = 0.75f;
    FARUtil::kTolerZ = 2.2f;
    FARUtil::IsDebug = false;
    MapHandlerParams params{};
    params.sensor_range = 30.0f;
    params.floor_height = 2.5f;
    params.cell_length = 5.0f;
    params.cell_height = 1.0f;
    params.grid_max_length = 1000.0f;
    params.grid_max_height = 100.0f;
    params.height_voxel_dim = 0.2f;
    std::cout << "case,calls,candidates,columns,retained,extended,milliseconds\n";
    const char* names[] = {"one_level", "five_levels", "map_corner", "no_terrain"};
    for (int scenario = 0; scenario < 4; ++scenario) {
        MapHandler map;
        map.Init(params);
        map.UpdateRobotPosition(Point3D(0.0f, 0.0f, 0.0f));
        const float offset = scenario == 2 ? 497.5f : 0.0f;
        const Point3D robot(offset, offset, 0.0f);
        map.UpdateRobotPosition(robot);
        PointCloudPtr ground(new PointCloud);
        PointCloudPtr heights(new PointCloud);
        if (scenario != 3) {
            for (int x = -32; x <= 32; ++x) {
                for (int y = -32; y <= 32; ++y) {
                    ground->push_back(FARUtil::Point3DToPCLPoint(Point3D(
                        robot.x + x * 0.8f, robot.y + y * 0.8f,
                        -0.75f + x * 0.0016f + y * 0.0008f)));
                }
            }
        }
        map.UpdateTerrainHeightGrid(ground, heights);
        if ((scenario != 3) == heights->empty()) return 2;
        map.UpdateRobotPosition(robot); // Restore the full, unfiltered candidates.
        auto candidates = MapHandlerTestAccess::Candidates();
        const auto& grid = MapHandlerTestAccess::World();
        if (scenario == 0) {
            const int level = grid.Pos2Sub(robot.x, robot.y, robot.z).z();
            for (auto it = candidates.begin(); it != candidates.end();) {
                if (grid.Ind2Sub(*it).z() == level) ++it;
                else it = candidates.erase(it);
            }
        }
        std::unordered_set<int> columns;
        const int stride = grid.GetSize().x() * grid.GetSize().y();
        for (int index : candidates) columns.insert(index % stride);
        std::unordered_set<int> expected, expected_extended;
        Reference(candidates, *heights, params.cell_length * 0.7071f,
                  params.cell_height, expected, expected_extended);
        double ms = 0.0;
        for (int i = -4; i < calls; ++i) {
            auto cells = candidates;
            std::unordered_set<int> extended;
            const auto start = std::chrono::steady_clock::now();
            MapHandlerTestAccess::Filter(map, cells, extended);
            const auto end = std::chrono::steady_clock::now();
            if (cells != expected || extended != expected_extended) {
                std::cerr << names[scenario] << ": terrain-neighbor sets differ\n";
                return 3;
            }
            if (i >= 0) ms += std::chrono::duration<double, std::milli>(end - start).count();
        }
        std::cout << names[scenario] << ',' << calls << ',' << candidates.size() << ','
                  << columns.size() << ',' << expected.size() << ',' << expected_extended.size()
                  << ',' << std::setprecision(10) << ms << '\n';
    }
}
