// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "far_planner/point_cloud_validation.h"
#include "far_planner/utility.h"

namespace
{
#ifdef FAR_PLANNER_ROS1
using TransformError = tf::TransformException;
using Stamp = ros::Time;
using CloudMessage = sensor_msgs::PointCloud2;
using CloudField = sensor_msgs::PointField;
#else
using TransformError = tf2::TransformException;
using Stamp = rclcpp::Time;
using CloudMessage = sensor_msgs::msg::PointCloud2;
using CloudField = sensor_msgs::msg::PointField;
#endif

CloudMessage ValidCloudMessage()
{
    CloudMessage cloud;
    cloud.width = cloud.height = 1;
    cloud.point_step = cloud.row_step = 16;
    cloud.data.resize(16);
    const std::uint16_t one = 1;
    cloud.is_bigendian = *reinterpret_cast<const unsigned char*>(&one) == 0;
    const char* names[] = {"x", "y", "z", "intensity"};
    for (std::size_t i = 0; i < 4; ++i) {
        CloudField field;
        field.name = names[i];
        field.offset = static_cast<std::uint32_t>(i * sizeof(float));
        field.datatype = CloudField::FLOAT32;
        field.count = 1;
        cloud.fields.push_back(field);
    }
    return cloud;
}

TEST(CloudLayout, RejectsTruncatedAndMisdeclaredBuffers)
{
    auto cloud = ValidCloudMessage();
    EXPECT_TRUE(far_planner::ValidPointCloudLayout(cloud, true));
    cloud.height = 2;
    cloud.row_step = 20;
    cloud.data.resize(40);  // Organized rows may include padding.
    EXPECT_TRUE(far_planner::ValidPointCloudLayout(cloud, true));

    cloud = ValidCloudMessage();
    cloud.data.clear();
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, true));
    cloud = ValidCloudMessage();
    cloud.data.pop_back();
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, true));
    cloud = ValidCloudMessage();
    cloud.row_step = 12;
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, true));
    cloud = ValidCloudMessage();
    cloud.fields[0].offset = 14;
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, true));
}

TEST(CloudLayout, RequiresCoordinatesAndTerrainClassification)
{
    auto cloud = ValidCloudMessage();
    cloud.fields.pop_back();
    EXPECT_TRUE(far_planner::ValidPointCloudLayout(cloud, false));
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, true));
    cloud = ValidCloudMessage();
    cloud.fields.erase(cloud.fields.begin());
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, false));
    cloud = ValidCloudMessage();
    cloud.fields[1].datatype = CloudField::FLOAT64;
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, false));
    cloud = ValidCloudMessage();
    cloud.is_bigendian = !cloud.is_bigendian;
    EXPECT_FALSE(far_planner::ValidPointCloudLayout(cloud, true));
}

TEST(CloudValues, TerrainDropsNonfiniteIntensity)
{
    PointCloudPtr cloud(new PointCloud);
    PCLPoint valid{};
    valid.x = 1.0f;
    valid.intensity = 0.5f;
    cloud->push_back(valid);
    PCLPoint invalid = valid;
    invalid.x = 2.0f;
    invalid.intensity = std::numeric_limits<float>::quiet_NaN();
    cloud->push_back(invalid);

    FARUtil::RemoveNanInfPoints(cloud, true);
    ASSERT_EQ(cloud->size(), 1u);
    EXPECT_FLOAT_EQ(cloud->front().x, 1.0f);
    EXPECT_EQ(cloud->width, cloud->size());
    EXPECT_EQ(cloud->height, 1u);

    cloud->push_back(invalid);
    FARUtil::RemoveNanInfPoints(cloud, false);
    ASSERT_EQ(cloud->size(), 2u);  // Scan intensity is optional.
    EXPECT_FLOAT_EQ(cloud->back().x, 2.0f);
    EXPECT_EQ(cloud->width, cloud->size());
    EXPECT_EQ(cloud->height, 1u);
}

TEST(CloudValues, HeightCropUpdatesCloudDimensions)
{
    PointCloudPtr cloud(new PointCloud);
    PCLPoint near{};
    near.z = FARUtil::robot_pos.z;
    cloud->push_back(near);
    PCLPoint far = near;
    far.z += 10.0f;
    cloud->push_back(far);

    FARUtil::CropCloudWithinHeight(cloud, 1.0f);
    ASSERT_EQ(cloud->size(), 1u);
    EXPECT_FLOAT_EQ(cloud->front().z, near.z);
    EXPECT_EQ(cloud->width, cloud->size());
    EXPECT_EQ(cloud->height, 1u);
}

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

// catkin_add_gtest links gtest without its main entry point.
int main(int argc, char** argv)
{
    ros::init(argc, argv, "far_planner_core_tests", ros::init_options::NoSigintHandler);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
