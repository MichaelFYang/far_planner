// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "far_planner/far_planner.h"

class LocalTerrainCallbackTestAccess {
public:
    static void VerifyPlanningRefreshesAddedNodes() {
#ifdef FAR_PLANNER_ROS1
        int argc = 0;
        char** argv = nullptr;
        if (!ros::isInitialized()) ros::init(argc, argv, "planning_callback_test");
#else
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
#endif
        FARMaster planner;
#ifndef FAR_PLANNER_ROS1
        struct ShutdownBeforePlanner {
            ~ShutdownBeforePlanner() { rclcpp::shutdown(); }
        } shutdown_before_planner;
#endif
        planner.graph_manager_.ResetCurrentGraph();
#ifndef FAR_PLANNER_ROS1
        planner.is_init_completed_ = true;
#endif
        planner.is_graph_init_ = true;
        NavNodePtr source, existing, added;
        DynamicGraph::CreateNavNodeFromPoint(Point3D(0, 0, 0), source, true);
        DynamicGraph::CreateNavNodeFromPoint(Point3D(1, 0, 0), existing, false);
        source->is_covered = existing->is_covered = true;
        DynamicGraph::AddNodeToGraph(source);
        DynamicGraph::AddNodeToGraph(existing);
        DynamicGraph::AddEdge(source, existing);
        planner.odom_node_ptr_ = source;
        planner.graph_planner_.UpdaetVGraph(DynamicGraph::GetNavGraph());

        // An import can add nodes and links between graph and planning ticks.
        DynamicGraph::CreateNavNodeFromPoint(Point3D(2, 0, 0), added, false);
        added->is_covered = true;
        DynamicGraph::AddNodeToGraph(added);
        DynamicGraph::AddEdge(existing, added);
#ifdef FAR_PLANNER_ROS1
        planner.PlanningCallBack(ros::TimerEvent());
#else
        planner.PlanningCallBack();
#endif
        EXPECT_TRUE(added->is_traversable);
        EXPECT_TRUE(added->is_free_traversable);
        EXPECT_FLOAT_EQ(added->gscore, 2.0f);
        EXPECT_FLOAT_EQ(added->fgscore, 2.0f);
        EXPECT_EQ(added->parent, existing);
        EXPECT_EQ(added->free_parent, existing);
        source->position.x = -1.0f;
#ifdef FAR_PLANNER_ROS1
        planner.PlanningCallBack(ros::TimerEvent());
#else
        planner.PlanningCallBack();
#endif
        EXPECT_FLOAT_EQ(added->gscore, 3.0f);
        EXPECT_FLOAT_EQ(added->fgscore, 3.0f);
    }

    static void VerifyTimingAndEmptyClear() {
#ifdef FAR_PLANNER_ROS1
        int argc = 0;
        char** argv = nullptr;
        if (!ros::isInitialized()) ros::init(argc, argv, "local_terrain_callback_test");
#else
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
#endif
        FARMaster planner;
#ifndef FAR_PLANNER_ROS1
        // Match main(): stop the listener's ROS context before destroying the planner.
        struct ShutdownBeforePlanner {
            ~ShutdownBeforePlanner() { rclcpp::shutdown(); }
        } shutdown_before_planner;
#endif
        planner.master_params_.world_frame = "map";
        planner.sensor_time_tolerance_ns_ = 200000000ULL;
        planner.local_terrain_ptr_.reset(new PointCloud);
        PCLPoint point{};
        point.x = 1.0f;
        planner.local_terrain_ptr_->push_back(point);
        ASSERT_TRUE(planner.odometry_history_.Add(10000000000ULL, {{0.0, 0.0, 0.0}}));

        const auto queue_clear = [&planner](std::uint64_t stamp) {
#ifdef FAR_PLANNER_ROS1
            sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
            cloud->header.stamp.fromNSec(stamp);
#else
            auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
            cloud->header.stamp.sec = static_cast<std::int32_t>(stamp / 1000000000ULL);
            cloud->header.stamp.nanosec = static_cast<std::uint32_t>(stamp % 1000000000ULL);
#endif
            cloud->header.frame_id = "sensor_without_tf";
            cloud->height = 1;
            return planner.pending_local_terrains_.Push(
                stamp, cloud, planner.last_local_terrain_stamp_, far_planner::SteadyNowNs());
        };

        ASSERT_TRUE(queue_clear(9000000000ULL));
        planner.ProcessPendingLocalTerrain();
        EXPECT_EQ(planner.pending_local_terrains_.Size(), 0U);
        EXPECT_EQ(planner.local_terrain_ptr_->size(), 1U);
        EXPECT_EQ(planner.local_terrain_revision_, 0U);

        ASSERT_TRUE(queue_clear(11000000000ULL));
        planner.ProcessPendingLocalTerrain();
        EXPECT_EQ(planner.pending_local_terrains_.Size(), 1U);
        EXPECT_EQ(planner.local_terrain_ptr_->size(), 1U);
        ASSERT_TRUE(planner.odometry_history_.Add(11000000000ULL, {{0.0, 0.0, 0.0}}));
        planner.ProcessPendingLocalTerrain();
        EXPECT_EQ(planner.pending_local_terrains_.Size(), 0U);
        EXPECT_TRUE(planner.local_terrain_ptr_->empty());
        EXPECT_EQ(planner.last_local_terrain_stamp_, 11000000000ULL);
        EXPECT_EQ(planner.local_terrain_revision_, 1U);

        // A newer odometry sample must not invalidate a still-queued cloud
        // whose measurement-time pose is retained in history.
        ASSERT_TRUE(planner.odometry_history_.Add(12000000000ULL, {{0.0, 0.0, 0.0}}));
        planner.local_terrain_ptr_->push_back(point);
        ASSERT_TRUE(queue_clear(12000000000ULL));
        ASSERT_TRUE(planner.odometry_history_.Add(13000000000ULL, {{0.0, 0.0, 0.0}}));
        planner.ProcessPendingLocalTerrain();
        EXPECT_EQ(planner.pending_local_terrains_.Size(), 0U);
        EXPECT_TRUE(planner.local_terrain_ptr_->empty());
        EXPECT_EQ(planner.last_local_terrain_stamp_, 12000000000ULL);
        EXPECT_EQ(planner.local_terrain_revision_, 2U);

        // A bracketing pose too far from the cloud is still rejected.
        ASSERT_TRUE(queue_clear(12500000000ULL));
        planner.ProcessPendingLocalTerrain();
        EXPECT_EQ(planner.last_local_terrain_stamp_, 12000000000ULL);
        EXPECT_EQ(planner.local_terrain_revision_, 2U);
    }
};

TEST(LocalTerrainCallback, WaitsForMatchingOdometryAndClearsWithoutTf) {
    LocalTerrainCallbackTestAccess::VerifyTimingAndEmptyClear();
}

TEST(PlanningCallback, RefreshesNodesAddedAfterGraphUpdate) {
    LocalTerrainCallbackTestAccess::VerifyPlanningRefreshesAddedNodes();
}
