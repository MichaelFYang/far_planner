// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "far_planner/far_planner.h"

class LocalTerrainCallbackTestAccess {
public:
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
