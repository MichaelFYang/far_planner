// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>

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

    // When the next terrain analysis yields no traversable cells, the old
    // index must not become a synthetic height sample at the world origin.
    map.UpdateTerrainHeightGrid(CloudAt(1000000.0f), terrain_height);
    EXPECT_TRUE(terrain_height->empty());
    float min_height = 0.0f, max_height = 0.0f;
    bool matched = true;
    const Point3D query(0.0f, 0.0f, 4.0f);
    EXPECT_FLOAT_EQ(MapHandler::NearestHeightOfRadius(query, 1.0f,
                                                      min_height, max_height, matched), 4.0f);
    EXPECT_FALSE(matched);
    EXPECT_FLOAT_EQ(map.TerrainHeightOfPoint(query, matched, true), 4.0f);
    EXPECT_FALSE(matched);
}

TEST(CloudUtilities, ConversionSkipsNonfiniteGeometryWithoutPadding) {
    PointStack points = {Point3D(1.0f, 2.0f, 3.0f, 4.0f),
                         Point3D(9.0f, 9.0f, std::numeric_limits<float>::quiet_NaN()),
                         Point3D(5.0f, 6.0f, 7.0f, 8.0f)};
    PointCloudPtr cloud(new PointCloud);
    FARUtil::ConvertCloudToPCL(points, cloud);
    ASSERT_EQ(cloud->size(), 2U);
    EXPECT_FLOAT_EQ(cloud->points[0].z, 3.0f);
    EXPECT_FLOAT_EQ(cloud->points[1].x, 5.0f);
    EXPECT_FLOAT_EQ(cloud->points[1].intensity, 8.0f);
}

TEST(CloudUtilities, ClearingKdTreePreservesEmptyCloudAndCanRebuild) {
    PointCloudPtr cloud(new PointCloud);
    FARUtil::UpdateKdTrees(cloud);
    EXPECT_TRUE(cloud->empty());
    EXPECT_FALSE(FARUtil::kdtree_new_cloud_->getInputCloud());
    EXPECT_EQ(FARUtil::PointInNewCounter(Point3D(0.0f, 0.0f, 0.0f), 1.0f), 0U);

    cloud->push_back(FARUtil::Point3DToPCLPoint(Point3D(0.0f, 0.0f, 0.0f)));
    FARUtil::UpdateKdTrees(cloud);
    EXPECT_EQ(FARUtil::PointInNewCounter(Point3D(0.0f, 0.0f, 0.0f), 1.0f), 1U);

    cloud->clear();
    FARUtil::UpdateKdTrees(cloud);
    EXPECT_TRUE(cloud->empty());
    EXPECT_FALSE(FARUtil::kdtree_new_cloud_->getInputCloud());
    EXPECT_EQ(FARUtil::PointInNewCounter(Point3D(0.0f, 0.0f, 0.0f), 1.0f), 0U);
}
