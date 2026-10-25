// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "far_planner/map_handler.h"

namespace {

struct RestoreMapParameters {
    float leaf = FARUtil::kLeafSize;
    float robot = FARUtil::robot_dim;
    float height = FARUtil::vehicle_height;
    Point3D position = FARUtil::robot_pos;
    ~RestoreMapParameters() {
        FARUtil::kLeafSize = leaf;
        FARUtil::robot_dim = robot;
        FARUtil::vehicle_height = height;
        FARUtil::robot_pos = position;
    }
};

PointCloudPtr CloudAt(float x) {
    PointCloudPtr cloud(new PointCloud);
    cloud->push_back(FARUtil::Point3DToPCLPoint(Point3D(x, 0.0f, 0.0f)));
    return cloud;
}

std::size_t CellSize(MapHandler& map, float x, CloudType type) {
    PointCloudPtr cloud(new PointCloud);
    map.GetCloudOfPoint(Point3D(x, 0.0f, 0.0f), cloud, type, false);
    return cloud->size();
}

}  // namespace

TEST(MapHandler, UpdatesAndRemovesOnlyAffectedCells) {
    RestoreMapParameters restore;
    FARUtil::kLeafSize = 0.2f;
    FARUtil::robot_dim = 0.8f;
    FARUtil::vehicle_height = 0.75f;

    MapHandlerParams params{};
    params.sensor_range = 6.0f;
    params.floor_height = 2.5f;
    params.cell_length = 2.0f;
    params.cell_height = 1.0f;
    params.grid_max_length = 20.0f;
    params.grid_max_height = 6.0f;
    params.height_voxel_dim = 0.2f;
    MapHandler map;
    map.Init(params);
    map.UpdateRobotPosition(Point3D(0.0f, 0.0f, 0.0f));

    auto obstacles = CloudAt(1.0f);
    obstacles->push_back(FARUtil::Point3DToPCLPoint(Point3D(5.0f, 0.0f, 0.0f)));
    map.UpdateObsCloudGrid(obstacles);
    EXPECT_EQ(CellSize(map, 1.0f, OBS_CLOUD), 1U);
    EXPECT_EQ(CellSize(map, 5.0f, OBS_CLOUD), 1U);

    map.RemoveObsCloudFromGrid(CloudAt(1.0f));
    EXPECT_EQ(CellSize(map, 1.0f, OBS_CLOUD), 0U);
    EXPECT_EQ(CellSize(map, 5.0f, OBS_CLOUD), 1U);

    map.UpdateObsCloudGrid(CloudAt(1.0f));
    map.RemoveObsCloudFromGrid(CloudAt(5.0f));
    EXPECT_EQ(CellSize(map, 1.0f, OBS_CLOUD), 1U);
    EXPECT_EQ(CellSize(map, 5.0f, OBS_CLOUD), 0U);

    map.UpdateFreeCloudGrid(CloudAt(1.0f));
    map.UpdateFreeCloudGrid(CloudAt(5.0f));
    EXPECT_EQ(CellSize(map, 1.0f, FREE_CLOUD), 1U);
    EXPECT_EQ(CellSize(map, 5.0f, FREE_CLOUD), 1U);

    PointStack occupied;
    map.GetOccupancyCeilsCenters(occupied);
    EXPECT_EQ(occupied.size(), 2U);
    map.ResetGripMapCloud();
    map.GetOccupancyCeilsCenters(occupied);
    EXPECT_TRUE(occupied.empty());
    EXPECT_EQ(CellSize(map, 1.0f, OBS_CLOUD), 0U);
    EXPECT_EQ(CellSize(map, 5.0f, FREE_CLOUD), 0U);
    map.UpdateObsCloudGrid(CloudAt(1.0f));
    map.GetOccupancyCeilsCenters(occupied);
    EXPECT_EQ(occupied.size(), 1U);

    // Traversability must use the pose assigned to this terrain update,
    // even if the planner's latest odometry has already advanced.
    FARUtil::robot_pos = Point3D(100.0f, 0.0f, 0.0f);
    auto ground = CloudAt(0.0f);
    ground->points.front().z = -FARUtil::vehicle_height;
    PointCloudPtr terrain_height(new PointCloud);
    map.UpdateTerrainHeightGrid(ground, terrain_height);
    EXPECT_FALSE(terrain_height->empty());
}
