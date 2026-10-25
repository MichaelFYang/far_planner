// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <limits>
#include "far_planner/sensor_timing.h"

using far_planner::OdometryHistory;
using far_planner::WithinSensorSkew;

TEST(SensorTiming, UsesExactOldPoseInsteadOfLatest)
{
    OdometryHistory history;
    ASSERT_TRUE(history.Add(100, {{1.0, 2.0, 3.0}}));
    ASSERT_TRUE(history.Add(200, {{3.0, 4.0, 5.0}}));
    OdometryHistory::Position position;
    ASSERT_TRUE(history.PositionAt(100, 50, position));
    EXPECT_DOUBLE_EQ(position[0], 1.0);
}

TEST(SensorTiming, InterpolatesOnlyBetweenNearbyMeasurements)
{
    OdometryHistory history;
    history.Add(100, {{1.0, 2.0, 3.0}});
    history.Add(200, {{3.0, 4.0, 5.0}});
    OdometryHistory::Position position{{9.0, 9.0, 9.0}};
    ASSERT_TRUE(history.PositionAt(150, 50, position));
    EXPECT_EQ(position, (OdometryHistory::Position{{2.0, 3.0, 4.0}}));
    EXPECT_FALSE(history.PositionAt(150, 49, position));
    EXPECT_FALSE(history.PositionAt(99, 100, position));
    EXPECT_FALSE(history.PositionAt(201, 100, position));
    EXPECT_EQ(position, (OdometryHistory::Position{{2.0, 3.0, 4.0}}));
}

TEST(SensorTiming, RejectsInvalidDuplicateAndOutOfOrderOdometry)
{
    OdometryHistory history;
    EXPECT_FALSE(history.Add(0, {{0.0, 0.0, 0.0}}));
    ASSERT_TRUE(history.Add(200, {{2.0, 0.0, 0.0}}));
    EXPECT_FALSE(history.Add(200, {{9.0, 0.0, 0.0}}));
    EXPECT_FALSE(history.Add(100, {{9.0, 0.0, 0.0}}));
    EXPECT_FALSE(history.Add(300, {{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}));
    EXPECT_EQ(history.LatestStamp(), 200U);
}

TEST(SensorTiming, BoundsHistoryAndCanStartANewEpoch)
{
    OdometryHistory history;
    for (std::uint64_t i = 1; i <= 2001; ++i) history.Add(i, {{0.0, 0.0, 0.0}});
    OdometryHistory::Position position;
    EXPECT_FALSE(history.PositionAt(1, 100, position));
    EXPECT_TRUE(history.PositionAt(2, 100, position));
    history.Add(10000002002ULL, {{0.0, 0.0, 0.0}});
    EXPECT_FALSE(history.PositionAt(2001, 100, position));
    history.Clear();
    EXPECT_EQ(history.LatestStamp(), 0U);
    EXPECT_TRUE(history.Add(1, {{0.0, 0.0, 0.0}}));
}

TEST(SensorTiming, SkewIsSymmetricAndRequiresBothMeasurements)
{
    EXPECT_TRUE(WithinSensorSkew(100, 120, 20));
    EXPECT_TRUE(WithinSensorSkew(120, 100, 20));
    EXPECT_FALSE(WithinSensorSkew(100, 121, 20));
    EXPECT_FALSE(WithinSensorSkew(0, 1, 20));
    EXPECT_FALSE(WithinSensorSkew(1, 0, 20));
}

TEST(SensorTiming, DetectsClockRewindIncludingZeroWithoutTreatingPauseAsReset)
{
    far_planner::ClockEpoch epoch;
    EXPECT_FALSE(epoch.Observe(100));
    EXPECT_FALSE(epoch.Observe(100));
    EXPECT_FALSE(epoch.Observe(200));
    EXPECT_TRUE(epoch.Observe(50));
    EXPECT_FALSE(epoch.Observe(60));
    EXPECT_TRUE(epoch.Observe(0));
    EXPECT_FALSE(epoch.Observe(0));
}

TEST(SensorTiming, PendingScanWaitsForLaterOdometryAndRejectsReordering)
{
    far_planner::PendingMeasurementQueue<int> scans;
    EXPECT_TRUE(scans.Push(150, 1, 100, 1000));
    EXPECT_FALSE(scans.HasReady(100));
    EXPECT_FALSE(scans.Push(150, 2, 100, 1000));
    EXPECT_FALSE(scans.Push(140, 3, 100, 1000));
    EXPECT_TRUE(scans.Push(170, 4, 100, 1001));
    ASSERT_TRUE(scans.HasReady(160));
    const auto first = scans.PopReady();
    EXPECT_EQ(first.stamp, 150U);
    EXPECT_EQ(first.message, 1);
    EXPECT_FALSE(scans.HasReady(160));
    EXPECT_TRUE(scans.HasReady(200));
    EXPECT_EQ(scans.PopReady().message, 4);
    EXPECT_FALSE(scans.Push(150, 5, 170, 1002));
}

TEST(SensorTiming, PendingScanQueueIsBoundedExpiresAndResets)
{
    far_planner::PendingMeasurementQueue<int> scans;
    for (std::uint64_t stamp = 1; stamp <= 33; ++stamp)
        EXPECT_TRUE(scans.Push(stamp, static_cast<int>(stamp), 0, stamp));
    EXPECT_EQ(scans.Size(), 32U);
    EXPECT_EQ(scans.PopReady().stamp, 2U);
    scans.DiscardExpired(100, 60);
    EXPECT_EQ(scans.Size(), 0U);
    EXPECT_TRUE(scans.Push(101, 7, 0, 100));
    scans.Clear();
    EXPECT_EQ(scans.Size(), 0U);
    EXPECT_TRUE(scans.Push(1, 8, 0, 101));
    scans.DiscardExpired(102, 1);
    EXPECT_EQ(scans.Size(), 1U);
}
