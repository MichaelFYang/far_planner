// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "far_planner/map_handler.h"
#include "far_planner/scan_handler.h"

class ScanHandlerTestAccess {
public:
    static void Trace(ScanHandler& scan, const Eigen::Vector3i& endpoint) {
        scan.SetRayCloud(endpoint);
    }
};

class MapHandlerTestAccess {
public:
    static void SetTerrainInflation(MapHandler& map, int cells) {
        map.INFLATE_N = cells;
    }
};

TEST(GridStorage, IndexCoordinatesRoundTripWithoutCachedSubscripts) {
    grid_ns::Grid<int> grid(Eigen::Vector3i(7, 5, 3), 0);
    for (int index = 0; index < grid.GetCellNumber(); ++index) {
        const auto sub = grid.Ind2Sub(index);
        EXPECT_EQ(grid.Sub2Ind(sub), index);
    }
}

TEST(GridStorage, RejectsInvalidDimensionsBeforeAllocating) {
    EXPECT_THROW((grid_ns::Grid<int>(Eigen::Vector3i(0, 2, 2), 0)), std::length_error);
    EXPECT_THROW((grid_ns::Grid<int>(Eigen::Vector3i(std::numeric_limits<int>::max(), 2, 1), 0)), std::length_error);
    EXPECT_THROW((grid_ns::Grid<int>(Eigen::Vector3i(2, 2, 2), 0,
                                     Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 0, 1))), std::invalid_argument);
}

TEST(GridStorage, RejectsExtremePositionsAndKeepsResolutionOnInvalidUpdate) {
    grid_ns::Grid<int> grid(Eigen::Vector3i(7, 5, 3), 0);
    const double huge = std::numeric_limits<double>::max();
    const auto positive = grid.Pos2Sub(huge, 0.0, 0.0);
    const auto negative = grid.Pos2Sub(-huge, 0.0, 0.0);
    const auto invalid = grid.Pos2Sub(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    EXPECT_FALSE(grid.InRange(positive));
    EXPECT_FALSE(grid.InRange(negative));
    EXPECT_FALSE(grid.InRange(invalid));
    EXPECT_GT(positive.x(), grid.GetSize().x());
    EXPECT_LT(negative.x(), -1);
    EXPECT_EQ(grid.Pos2Sub(-1e-8, 0.0, 0.0).x(), 0);
    EXPECT_THROW(grid.SetResolution(Eigen::Vector3d(1.0, 0.0, 1.0)), std::invalid_argument);
    EXPECT_DOUBLE_EQ(grid.GetResolution().y(), 1.0);
}

TEST(GridStorage, RayTraceUsesWideDistanceMath) {
    grid_ns::Grid<int, true> grid(Eigen::Vector3i(50002, 1, 1), 0);
    std::vector<Eigen::Vector3i> cells;
    grid.RayTraceSubs(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(50000, 0, 0), cells);
    ASSERT_EQ(cells.size(), 50001U);
    EXPECT_EQ(cells.back().x(), 50000);
    grid.RayTraceSubs(Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(0, 0, 0), cells);
    EXPECT_TRUE(cells.empty());
}

TEST(GridStorage, SparseCellsMaterializeOnlyWhenWritten) {
    grid_ns::Grid<int, true> grid(Eigen::Vector3i(10000, 10000, 1), 0);
    EXPECT_EQ(grid.GetCellNumber(), 100000000);
    EXPECT_EQ(grid.MaterializedCellCount(), 0U);
    EXPECT_EQ(grid.GetCellValue(99999999), 0);
    EXPECT_EQ(grid.MaterializedCellCount(), 0U);
    grid.GetCell(99999999) = 7;
    EXPECT_EQ(grid.GetCellValue(99999999), 7);
    EXPECT_EQ(grid.MaterializedCellCount(), 1U);
    grid.EraseCell(99999999);
    EXPECT_EQ(grid.MaterializedCellCount(), 0U);
    EXPECT_EQ(grid.GetCellValue(99999999), 0);
}

namespace {

struct RestoreMapParameters {
    float leaf = FARUtil::kLeafSize;
    float robot = FARUtil::robot_dim;
    float height = FARUtil::vehicle_height;
    int inflate = FARUtil::kObsInflate;
    Point3D position = FARUtil::robot_pos;
    ~RestoreMapParameters() {
        FARUtil::kLeafSize = leaf;
        FARUtil::robot_dim = robot;
        FARUtil::vehicle_height = height;
        FARUtil::kObsInflate = inflate;
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

TEST(ScanHandler, EmptyInputsClearPriorOutputs) {
    ScanHandler scan;
    PointCloudPtr empty(new PointCloud);
    auto output = CloudAt(1.0f);
    scan.ExtractDyObsCloud(empty, output);
    EXPECT_TRUE(output->empty());
    output->push_back(FARUtil::Point3DToPCLPoint(Point3D(1.0f, 0.0f, 0.0f)));
    scan.GridVisualCloud(output, GridStatus::RAY);
    EXPECT_TRUE(output->empty());

    ScanHandlerParams params{};
    params.terrain_range = 2.0f;
    params.voxel_size = 1.0f;
    params.ceil_height = 2.0f;
    scan.Init(params);
    output->push_back(FARUtil::Point3DToPCLPoint(Point3D(1.0f, 0.0f, 0.0f)));
    scan.ExtractDyObsCloud(empty, output);
    EXPECT_TRUE(output->empty());
}

TEST(ScanHandler, RayTracingCoversEachDominantAxisAndLargeOffsets) {
    ScanHandlerParams params{};
    params.terrain_range = 4.0f;
    params.voxel_size = 1.0f;
    params.ceil_height = 4.0f;
    ScanHandler scan;
    scan.Init(params);
    scan.UpdateRobotPosition(Point3D(0.0f, 0.0f, 0.0f));
    PointCloudPtr rays(new PointCloud);
    const auto check = [&](const Eigen::Vector3i& endpoint,
                           const std::vector<Point3D>& expected) {
        scan.ReInitGrids();
        ScanHandlerTestAccess::Trace(scan, endpoint);
        scan.GridVisualCloud(rays, GridStatus::RAY);
        ASSERT_EQ(rays->size(), expected.size());
        for (std::size_t i = 0; i < expected.size(); ++i) {
            EXPECT_FLOAT_EQ(rays->at(i).x, expected[i].x);
            EXPECT_FLOAT_EQ(rays->at(i).y, expected[i].y);
            EXPECT_FLOAT_EQ(rays->at(i).z, expected[i].z);
        }
    };
    check(Eigen::Vector3i(7, 5, 4),
          {Point3D(1.0f, 0.0f, 0.0f), Point3D(2.0f, 0.0f, 0.0f)});
    check(Eigen::Vector3i(5, 7, 4),
          {Point3D(0.0f, 1.0f, 0.0f), Point3D(0.0f, 2.0f, 0.0f)});
    check(Eigen::Vector3i(5, 4, 7),
          {Point3D(0.0f, 0.0f, 1.0f), Point3D(0.0f, 0.0f, 2.0f)});
    // Squaring a 65536-cell offset overflowed the previous int guard.
    check(Eigen::Vector3i(65540, 4, 4),
          {Point3D(1.0f, 0.0f, 0.0f), Point3D(2.0f, 0.0f, 0.0f),
           Point3D(3.0f, 0.0f, 0.0f), Point3D(4.0f, 0.0f, 0.0f)});
}

TEST(ScanHandler, DistantCorruptPointDoesNotInflateLocalGrid) {
    RestoreMapParameters restore;
    FARUtil::kObsInflate = 0;
    ScanHandlerParams params{};
    params.terrain_range = 2.0f;
    params.voxel_size = 1.0f;
    params.ceil_height = 2.0f;
    ScanHandler scan;
    scan.Init(params);
    scan.UpdateRobotPosition(Point3D(0.0f, 0.0f, 0.0f));
    PointCloudPtr free_cloud(new PointCloud);
    auto near_cloud = CloudAt(1.0f);
    scan.SetCurrentScanCloud(near_cloud, free_cloud);
    ASSERT_EQ(near_cloud->size(), 1U);
    EXPECT_FLOAT_EQ(near_cloud->front().x, 1.0f);
    PointCloudPtr baseline(new PointCloud);
    scan.GridVisualCloud(baseline, GridStatus::SCAN);
    ASSERT_FALSE(baseline->empty());
    EXPECT_EQ(baseline->width, baseline->size());
    EXPECT_EQ(baseline->height, 1U);

    scan.ReInitGrids();
    auto mixed_cloud = CloudAt(1.0f);
    mixed_cloud->push_back(FARUtil::Point3DToPCLPoint(Point3D(1e30f, 0.0f, 0.0f)));
    scan.SetCurrentScanCloud(mixed_cloud, free_cloud);
    PointCloudPtr actual(new PointCloud);
    scan.GridVisualCloud(actual, GridStatus::SCAN);
    EXPECT_EQ(actual->size(), baseline->size());
    EXPECT_EQ(actual->width, actual->size());
}

TEST(MapHandler, RejectsInvalidDerivedGridDimensions) {
    RestoreMapParameters restore;
    FARUtil::robot_dim = 0.8f;
    MapHandlerParams params{};
    params.sensor_range = 6.0f;
    params.cell_length = 0.0f;
    params.cell_height = 1.0f;
    params.grid_max_length = 20.0f;
    params.grid_max_height = 6.0f;
    MapHandler map;
    EXPECT_THROW(map.Init(params), std::invalid_argument);

    params.cell_length = 1.0f;
    params.grid_max_length = 1e30f;
    EXPECT_THROW(map.Init(params), std::invalid_argument);
}

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
    EXPECT_EQ(obstacles->width, obstacles->size());
    EXPECT_EQ(obstacles->height, 1U);
    EXPECT_EQ(CellSize(map, 1.0f, OBS_CLOUD), 1U);
    EXPECT_EQ(CellSize(map, 5.0f, OBS_CLOUD), 1U);

    map.RemoveObsCloudFromGrid(CloudAt(1.0f));
    EXPECT_EQ(CellSize(map, 1.0f, OBS_CLOUD), 0U);
    EXPECT_EQ(CellSize(map, 5.0f, OBS_CLOUD), 1U);
    PointStack after_removal;
    map.GetOccupancyCeilsCenters(after_removal);
    EXPECT_EQ(after_removal.size(), 1U);

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
    PointStack old_neighbors{Point3D(1.0f, 0.0f, 0.0f)};
    map.GetNeighborCeilsCenters(old_neighbors);
    EXPECT_TRUE(old_neighbors.empty());
    auto old_obstacles = CloudAt(1.0f);
    auto old_free = CloudAt(1.0f);
    map.GetSurroundObsCloud(old_obstacles);
    map.GetSurroundFreeCloud(old_free);
    EXPECT_TRUE(old_obstacles->empty());
    EXPECT_TRUE(old_free->empty());
    EXPECT_EQ(CellSize(map, 1.0f, OBS_CLOUD), 0U);
    EXPECT_EQ(CellSize(map, 5.0f, FREE_CLOUD), 0U);
    map.UpdateRobotPosition(Point3D(0.0f, 0.0f, 0.0f));
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
    EXPECT_EQ(terrain_height->width, terrain_height->size());
    EXPECT_EQ(terrain_height->height, 1U);

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

TEST(MapHandler, FindsConnectedGroundAfterSearchingFromEmptyRobotCell) {
    RestoreMapParameters restore;
    FARUtil::kLeafSize = 0.2f;
    FARUtil::robot_dim = 1.0f;
    FARUtil::vehicle_height = 0.75f;
    MapHandlerParams params{};
    params.sensor_range = 3.0f;
    params.floor_height = 2.5f;
    params.cell_length = 1.0f;
    params.cell_height = 1.0f;
    params.grid_max_length = 10.0f;
    params.grid_max_height = 6.0f;
    params.height_voxel_dim = 0.2f;
    MapHandler map;
    map.Init(params);
    map.UpdateRobotPosition(Point3D(0.0f, 0.0f, 0.0f));
    MapHandlerTestAccess::SetTerrainInflation(map, 0);
    PointCloudPtr ground(new PointCloud);
    ground->push_back(FARUtil::Point3DToPCLPoint(Point3D(1.0f, 0.0f, -0.75f)));
    ground->push_back(FARUtil::Point3DToPCLPoint(Point3D(1.0f, 1.0f, -0.75f)));
    PointCloudPtr heights(new PointCloud);
    map.UpdateTerrainHeightGrid(ground, heights);
    EXPECT_EQ(heights->size(), 2U);
}

TEST(MapHandler, EmptyTerrainAndResetDiscardOldHeightSamples) {
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

    auto ground = CloudAt(0.0f);
    ground->points.front().z = -FARUtil::vehicle_height;
    PointCloudPtr terrain_height(new PointCloud);
    map.UpdateTerrainHeightGrid(ground, terrain_height);
    ASSERT_FALSE(terrain_height->empty());
    const Point3D query(terrain_height->front().x, terrain_height->front().y, 4.0f);
    float min_height = 0.0f, max_height = 0.0f;
    bool matched = false;
    MapHandler::NearestHeightOfRadius(query, 0.1f, min_height, max_height, matched);
    ASSERT_TRUE(matched);

    PointCloudPtr empty(new PointCloud);
    map.UpdateTerrainHeightGrid(empty, terrain_height);
    EXPECT_TRUE(terrain_height->empty());
    EXPECT_FLOAT_EQ(MapHandler::NearestHeightOfRadius(query, 0.1f,
                                                     min_height, max_height, matched), query.z);
    EXPECT_FALSE(matched);
    EXPECT_FLOAT_EQ(map.TerrainHeightOfPoint(query, matched, true), query.z);
    EXPECT_FALSE(matched);

    map.UpdateTerrainHeightGrid(ground, terrain_height);
    ASSERT_FALSE(terrain_height->empty());
    MapHandler::NearestHeightOfRadius(query, 0.1f, min_height, max_height, matched);
    ASSERT_TRUE(matched);
    map.ResetGripMapCloud();
    EXPECT_FLOAT_EQ(MapHandler::NearestHeightOfRadius(query, 0.1f,
                                                     min_height, max_height, matched), query.z);
    EXPECT_FALSE(matched);
    EXPECT_FLOAT_EQ(map.TerrainHeightOfPoint(query, matched, true), query.z);
    EXPECT_FALSE(matched);
}

TEST(MapHandler, ResetRecentersAtNextOdometry) {
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
    map.ResetGripMapCloud();

    map.UpdateRobotPosition(Point3D(1000.0f, 0.0f, 0.0f));
    auto obstacle = CloudAt(1000.0f);
    map.UpdateObsCloudGrid(obstacle);
    EXPECT_EQ(CellSize(map, 1000.0f, OBS_CLOUD), 1U);
    EXPECT_EQ(obstacle->size(), 1U);
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

TEST(CloudUtilities, ClusterFilterDoesNotRetainAnIndexForReplacedPoints) {
    PointCloudPtr cloud(new PointCloud);
    cloud->push_back(FARUtil::Point3DToPCLPoint(Point3D(0.0f, 0.0f, 0.0f)));
    cloud->push_back(FARUtil::Point3DToPCLPoint(Point3D(0.1f, 0.0f, 0.0f)));
    cloud->push_back(FARUtil::Point3DToPCLPoint(Point3D(10.0f, 0.0f, 0.0f)));
    FARUtil::ClusterFilterCloud(cloud, 0.5f, 1);
    ASSERT_EQ(cloud->size(), 2U);
    EXPECT_FLOAT_EQ(cloud->points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(cloud->points[1].x, 0.1f);
    EXPECT_FALSE(FARUtil::kdtree_filter_cloud_->getInputCloud());

    cloud->clear();
    FARUtil::ClusterFilterCloud(cloud, 0.5f, 1);
    EXPECT_FALSE(FARUtil::kdtree_filter_cloud_->getInputCloud());
}

TEST(CloudUtilities, DecayKeepsRecentPointsInOrder) {
#ifdef FAR_PLANNER_ROS1
    ros::Time::init();
    const double now = ros::Time::now().toSec();
#else
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("cloud_decay_test");
    const double now = node->now().seconds();
#endif
    const double previous_start = FARUtil::systemStartTime;
    FARUtil::systemStartTime = now - 10.0;
    PointCloudPtr stack(new PointCloud);
    stack->push_back(FARUtil::Point3DToPCLPoint(Point3D(1.0f, 0.0f, 0.0f, 3.0f)));
    stack->push_back(FARUtil::Point3DToPCLPoint(Point3D(2.0f, 0.0f, 0.0f, 9.0f)));
    stack->push_back(FARUtil::Point3DToPCLPoint(Point3D(3.0f, 0.0f, 0.0f, 4.0f)));
    PointCloudPtr current(new PointCloud);
    current->push_back(FARUtil::Point3DToPCLPoint(Point3D(4.0f, 0.0f, 0.0f, -1.0f)));
#ifdef FAR_PLANNER_ROS1
    FARUtil::StackCloudByTime(current, stack, 5.0f);
#else
    FARUtil::StackCloudByTime(current, stack, 5.0f, node);
#endif
    // A bad negative duration must not overrun an index list after appending.
    PointCloudPtr invalid_window(new PointCloud);
    PointCloudPtr one_point(new PointCloud);
    one_point->push_back(FARUtil::Point3DToPCLPoint(Point3D(5.0f, 0.0f, 0.0f)));
#ifdef FAR_PLANNER_ROS1
    FARUtil::StackCloudByTime(one_point, invalid_window, -1.0f);
#else
    FARUtil::StackCloudByTime(one_point, invalid_window, -1.0f, node);
#endif
    FARUtil::systemStartTime = previous_start;
    EXPECT_TRUE(invalid_window->empty());
    ASSERT_EQ(stack->size(), 2U);
    EXPECT_FLOAT_EQ(stack->points[0].x, 2.0f);
    EXPECT_FLOAT_EQ(stack->points[1].x, 4.0f);
    EXPECT_FLOAT_EQ(stack->points[1].intensity, current->points[0].intensity);
    EXPECT_GE(current->points[0].intensity, 9.0f);
    EXPECT_LT(current->points[0].intensity, 11.0f);
}
