// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "far_planner/far_planner.h"

class ViewpointExtensionTestAccess {
public:
    static void VerifyClearLocalCloudStillExtendsViewpoint()
    {
#ifdef FAR_PLANNER_ROS1
        int argc = 0;
        char** argv = nullptr;
        if (!ros::isInitialized()) ros::init(argc, argv, "viewpoint_extension_test");
#else
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
#endif
        struct NearDistGuard {
            float previous = FARUtil::kNearDist;
            ~NearDistGuard() { FARUtil::kNearDist = previous; }
        } near_dist_guard;
        FARUtil::kNearDist = 1.0f;

        FARMaster planner;
#ifndef FAR_PLANNER_ROS1
        struct ShutdownBeforePlanner {
            ~ShutdownBeforePlanner() { rclcpp::shutdown(); }
        } shutdown_before_planner;
#endif
        planner.robot_pos_ = Point3D(0.0f, 0.0f, 0.0f);
        planner.viewpoint_around_ptr_.reset(new PointCloud);
        auto nav = std::make_shared<NavNode>();
        nav->position = Point3D(3.0f, 0.0f, 0.0f);
        nav->free_direct = NodeFreeDirect::CONVEX;
        nav->surf_dirs = {Point3D(-1.0f, 0.0f, 0.0f), Point3D(-1.0f, 0.0f, 0.0f)};
        PointCloudPtr obstacles(new PointCloud);
        PCLPoint far_obstacle{};
        far_obstacle.x = 100.0f;
        obstacles->push_back(far_obstacle);

        float free_dist = 4.0f;
        const Point3D waypoint = planner.ExtendViewpointOnObsCloud(nav, obstacles, free_dist);
        EXPECT_TRUE(planner.viewpoint_around_ptr_->empty());
        EXPECT_FLOAT_EQ(waypoint.x, 5.0f);
        EXPECT_FLOAT_EQ(waypoint.y, 0.0f);
        EXPECT_FLOAT_EQ(waypoint.z, 0.0f);
        EXPECT_FLOAT_EQ(free_dist, 4.0f);
    }
};

TEST(ViewpointExtension, ExtendsWhenInputObstaclesAreOutsideTheLocalCrop)
{
    ViewpointExtensionTestAccess::VerifyClearLocalCloudStillExtendsViewpoint();
}
