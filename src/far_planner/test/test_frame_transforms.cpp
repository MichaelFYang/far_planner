// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "far_planner/utility.h"

namespace
{
#ifdef FAR_PLANNER_ROS1
using TransformError = tf::TransformException;
using Stamp = ros::Time;
#else
using TransformError = tf2::TransformException;
using Stamp = rclcpp::Time;
#endif

class FrameTransforms : public ::testing::Test
{
protected:
    void SetUp() override
    {
#ifdef FAR_PLANNER_ROS1
        ros::Time::init();
        buffer = std::make_shared<tf::Transformer>();
#else
        buffer = std::make_shared<tf2_ros::Buffer>(
            std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
#endif
        AddTransform(10, 1.0);
        AddTransform(12, 3.0);
    }

    void AddTransform(int seconds, double x)
    {
#ifdef FAR_PLANNER_ROS1
        tf::Transform transform;
        transform.setIdentity();
        transform.setOrigin(tf::Vector3(x, 0.0, 0.0));
        ASSERT_TRUE(buffer->setTransform(
            tf::StampedTransform(transform, Stamp(seconds, 0), "map", "sensor"), "test"));
#else
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.child_frame_id = "sensor";
        transform.header.stamp.sec = seconds;
        transform.transform.translation.x = x;
        transform.transform.rotation.w = 1.0;
        ASSERT_TRUE(buffer->setTransform(transform, "test"));
#endif
    }

    PointCloudPtr Cloud(int seconds)
    {
        auto cloud = PointCloudPtr(new PointCloud);
        cloud->header.frame_id = "sensor";
        // PCL timestamps are expressed in microseconds.
        cloud->header.stamp = static_cast<std::uint64_t>(seconds) * 1000000ULL;
        PCLPoint point;
        point.x = 2.0f;
        point.y = 1.0f;
        point.z = 0.5f;
        point.intensity = 7.0f;
        cloud->push_back(point);
        return cloud;
    }

#ifdef FAR_PLANNER_ROS1
    tf::Transformer* Handle() { return buffer.get(); }
    std::shared_ptr<tf::Transformer> buffer;
#else
    std::shared_ptr<tf2_ros::Buffer> Handle() { return buffer; }
    std::shared_ptr<tf2_ros::Buffer> buffer;
#endif
};

TEST_F(FrameTransforms, UsesCloudMeasurementTimeInsteadOfLatestTransform)
{
    auto cloud = Cloud(10);
    FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(10, 0));
    EXPECT_FLOAT_EQ(cloud->front().x, 3.0f);
    EXPECT_FLOAT_EQ(cloud->front().intensity, 7.0f);
    EXPECT_EQ(cloud->header.frame_id, "map");
    EXPECT_EQ(cloud->header.stamp, 10000000ULL);
}

TEST_F(FrameTransforms, ReturnsSensorOriginFromTheSameMeasurementTime)
{
    auto cloud = Cloud(10);
    Point3D origin(99.0f, 99.0f, 99.0f);
    FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(10, 0), &origin);
    EXPECT_FLOAT_EQ(cloud->front().x, 3.0f);
    EXPECT_FLOAT_EQ(origin.x, 1.0f);
    EXPECT_FLOAT_EQ(origin.y, 0.0f);
    EXPECT_FLOAT_EQ(origin.z, 0.0f);

    cloud = Cloud(11);
    FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(11, 0), &origin);
    EXPECT_FLOAT_EQ(origin.x, 2.0f);
}

TEST_F(FrameTransforms, KeepsFallbackOriginForWorldFrameCloud)
{
    auto cloud = Cloud(10);
    Point3D origin(7.0f, 8.0f, 9.0f);
    FARUtil::TransformPCLFrame("map", "map", Handle(), cloud, Stamp(10, 0), &origin);
    EXPECT_FLOAT_EQ(origin.x, 7.0f);
    EXPECT_FLOAT_EQ(origin.y, 8.0f);
    EXPECT_FLOAT_EQ(origin.z, 9.0f);
}

TEST_F(FrameTransforms, FailedTfLeavesSensorOriginUnchanged)
{
    auto cloud = Cloud(13);
    Point3D origin(7.0f, 8.0f, 9.0f);
    EXPECT_THROW(FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(13, 0), &origin),
                 TransformError);
    EXPECT_FLOAT_EQ(origin.x, 7.0f);
    EXPECT_FLOAT_EQ(origin.y, 8.0f);
    EXPECT_FLOAT_EQ(origin.z, 9.0f);
}

TEST_F(FrameTransforms, InterpolatesAtCloudMeasurementTime)
{
    auto cloud = Cloud(11);
    FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(11, 0));
    EXPECT_FLOAT_EQ(cloud->front().x, 4.0f);
    EXPECT_EQ(cloud->header.stamp, 11000000ULL);
}

TEST_F(FrameTransforms, RetainsOriginalRosTimestampPrecisionForTheLookup)
{
    auto cloud = Cloud(12);
    // One nanosecond after the newest TF sample must not round down to that sample.
    EXPECT_THROW(FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(12, 1)),
                 TransformError);
    EXPECT_FLOAT_EQ(cloud->front().x, 2.0f);
}

TEST_F(FrameTransforms, RejectsExtrapolationWithoutMutatingTheInput)
{
    for (int stamp : {9, 13}) {
        auto cloud = Cloud(stamp);
        EXPECT_THROW(FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(stamp, 0)),
                     TransformError);
        EXPECT_FLOAT_EQ(cloud->front().x, 2.0f);
        EXPECT_EQ(cloud->header.frame_id, "sensor");
        EXPECT_EQ(cloud->header.stamp, static_cast<std::uint64_t>(stamp) * 1000000ULL);
    }
}

TEST_F(FrameTransforms, RejectsUnstampedSensorCloud)
{
    auto cloud = Cloud(0);
    EXPECT_THROW(FARUtil::TransformPCLFrame("sensor", "map", Handle(), cloud, Stamp(0, 0)),
                 TransformError);
    EXPECT_FLOAT_EQ(cloud->front().x, 2.0f);
}

TEST_F(FrameTransforms, UnknownGoalFramePropagatesFailure)
{
    Point3D goal(1.0f, 2.0f, 3.0f);
    EXPECT_THROW(FARUtil::TransformPoint3DFrame("missing", "map", Handle(), goal, Stamp(10, 0)),
                 TransformError);
    EXPECT_FLOAT_EQ(goal.x, 1.0f);
    EXPECT_FLOAT_EQ(goal.y, 2.0f);
    EXPECT_FLOAT_EQ(goal.z, 3.0f);
}

TEST_F(FrameTransforms, UsesGoalMeasurementTime)
{
    Point3D goal(1.0f, 2.0f, 3.0f);
    FARUtil::TransformPoint3DFrame("sensor", "map", Handle(), goal, Stamp(10, 0));
    EXPECT_FLOAT_EQ(goal.x, 2.0f);
}

TEST_F(FrameTransforms, ZeroStampedOperatorGoalExplicitlyUsesLatest)
{
    Point3D goal(1.0f, 2.0f, 3.0f);
    FARUtil::TransformPoint3DFrame("sensor", "map", Handle(), goal, Stamp(0, 0));
    EXPECT_FLOAT_EQ(goal.x, 4.0f);
}

TEST_F(FrameTransforms, NormalizesLegacyLeadingSlashes)
{
    auto cloud = Cloud(10);
    FARUtil::TransformPCLFrame("/sensor", "/map", Handle(), cloud, Stamp(10, 0));
    EXPECT_FLOAT_EQ(cloud->front().x, 3.0f);
    EXPECT_EQ(cloud->header.frame_id, "map");
}

TEST_F(FrameTransforms, IdentityTransformDoesNotRequireATfTree)
{
    auto cloud = Cloud(10);
    cloud->header.stamp = 42; // An unrelated PCL stamp must not escape.
    FARUtil::TransformPCLFrame("/map", "map", Handle(), cloud, Stamp(10, 123456789));
    EXPECT_FLOAT_EQ(cloud->front().x, 2.0f);
    EXPECT_EQ(cloud->header.frame_id, "map");
    EXPECT_EQ(cloud->header.stamp, 10123456ULL);
}

TEST_F(FrameTransforms, EmptyFrameIsRejectedWithoutMutatingTheInput)
{
    auto cloud = Cloud(10);
    EXPECT_THROW(FARUtil::TransformPCLFrame("", "map", Handle(), cloud, Stamp(10, 0)),
                 TransformError);
    EXPECT_FLOAT_EQ(cloud->front().x, 2.0f);
}

TEST(FrameNames, EmptyFramesNeverMatch)
{
    EXPECT_FALSE(FARUtil::IsSameFrameID("", ""));
    EXPECT_FALSE(FARUtil::IsSameFrameID("/", ""));
    EXPECT_FALSE(FARUtil::IsSameFrameID("map", ""));
    EXPECT_TRUE(FARUtil::IsSameFrameID("/map", "map"));
    EXPECT_FALSE(FARUtil::IsSameFrameID("map", "odom"));
}
}  // namespace
