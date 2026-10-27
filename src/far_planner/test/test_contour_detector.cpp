// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

#include "far_planner/contour_detector.h"

class ContourDetectorTestAccess {
public:
    static std::vector<unsigned char> CollectHierarchy(const std::vector<cv::Vec4i>& hierarchy,
                                                       int first,
                                                       std::vector<unsigned char> remove = {}) {
        ContourDetector detector;
        if (remove.empty()) remove.resize(hierarchy.size(), 0);
        detector.SameAndLowLevelIdxs(hierarchy, first, remove);
        return remove;
    }

    static void FilterContours(std::vector<CVPointStack>& contours,
                               const std::vector<cv::Vec4i>& hierarchy) {
        ContourDetector detector;
        detector.free_odom_resized_ = cv::Point2f(0.0f, 0.0f);
        detector.refined_hierarchy_ = hierarchy;
        detector.TopoFilterContours(contours);
    }

    static std::size_t CollapseWallRun(std::size_t count) {
        ContourDetector detector;
        detector.ALIGN_ANGLE_COS = 0.99f;
        CVPointStack contour;
        contour.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            contour.emplace_back(static_cast<float>(i), 0.0f);
        }
        detector.RemoveWallConnection(contour, cv::Point2f(-1.0f, 0.0f), count);
        return count;
    }

    static bool DegenerateWallVertex() {
        ContourDetector detector;
        return detector.IsPrevWallVertex(cv::Point2f(0.0f, 0.0f),
                                         cv::Point2f(0.0f, 0.0f),
                                         cv::Point2f(1.0f, 0.0f));
    }
};

TEST(ContourDetector, TraversesDeepHierarchyAndWallRunWithoutRecursion) {
    constexpr int depth = 100000;
    std::vector<cv::Vec4i> hierarchy(depth, cv::Vec4i(-1, -1, -1, -1));
    for (int i = 0; i + 1 < depth; ++i) hierarchy[i][2] = i + 1;
    const auto descendants = ContourDetectorTestAccess::CollectHierarchy(hierarchy, 0);
    EXPECT_EQ(std::count(descendants.begin(), descendants.end(), 1), depth);
    EXPECT_EQ(descendants.back(), 1U);
    EXPECT_EQ(ContourDetectorTestAccess::CollapseWallRun(depth), 1U);
    EXPECT_FALSE(ContourDetectorTestAccess::DegenerateWallVertex());

    std::vector<cv::Vec4i> siblings(4, cv::Vec4i(-1, -1, -1, -1));
    siblings[1][0] = 2;
    siblings[2][2] = 3;
    const auto branch = ContourDetectorTestAccess::CollectHierarchy(siblings, 1);
    EXPECT_EQ(branch, (std::vector<unsigned char>{0, 1, 1, 1}));
    const auto premarked = ContourDetectorTestAccess::CollectHierarchy(siblings, 2, {0, 0, 1, 0});
    EXPECT_EQ(premarked, (std::vector<unsigned char>{0, 0, 1, 1}));
}

TEST(ContourDetector, KeepsParentAndRemovesMarkedChildrenInOriginalOrder) {
    const CVPointStack outer{{-10.0f, -10.0f}, {10.0f, -10.0f},
                             {10.0f, 10.0f}, {-10.0f, 10.0f}};
    const CVPointStack outside{{2.0f, 2.0f}, {3.0f, 2.0f}, {2.0f, 3.0f}};
    const CVPointStack child{{2.2f, 2.2f}, {2.4f, 2.2f}, {2.2f, 2.4f}};
    std::vector<CVPointStack> contours{outer, outside, child, {{1.0f, 1.0f}}};
    std::vector<cv::Vec4i> hierarchy(4, cv::Vec4i(-1, -1, -1, -1));
    hierarchy[0][2] = 1;
    hierarchy[1][2] = 2;
    ContourDetectorTestAccess::FilterContours(contours, hierarchy);
    ASSERT_EQ(contours.size(), 2U);
    EXPECT_EQ(contours[0], outer);
    EXPECT_EQ(contours[1], outside);
}

TEST(ContourDetector, KeepsInflatedRectangleAfterContourFiltering)
{
    struct RestoreGlobals {
        Point3D free_odom = FARUtil::free_odom_p;
        bool static_env = FARUtil::IsStaticEnv;
        ~RestoreGlobals() {
            FARUtil::free_odom_p = free_odom;
            FARUtil::IsStaticEnv = static_env;
        }
    } restore;
    FARUtil::free_odom_p = Point3D(0.0f, 0.0f, 0.0f);
    FARUtil::IsStaticEnv = true;

    ContourDetectParams params{};
    params.sensor_range = 5.0f;
    params.voxel_dim = 1.0f;
    params.kRatio = 1.0f;
    params.kThredValue = 0;
    params.kBlurSize = 1;
    params.is_save_img = false;
    ContourDetector detector;
    detector.Init(params);
    auto odom = std::make_shared<NavNode>();
    odom->position = Point3D(0.0f, 0.0f, 0.0f);
    PointCloudPtr obstacles(new PointCloud);
    for (int offset = -2; offset <= 2; ++offset) {
        for (const auto& xy : {std::pair<int, int>(offset, -2), {offset, 2},
                               {-2, offset}, {2, offset}}) {
            PCLPoint point{};
            point.x = static_cast<float>(xy.first);
            point.y = static_cast<float>(xy.second);
            obstacles->push_back(point);
        }
    }
    std::vector<PointStack> contours;
    detector.BuildTerrainImgAndExtractContour(odom, obstacles, contours);
    ASSERT_EQ(contours.size(), 1U);
    ASSERT_EQ(contours.front().size(), 4U);
    const std::array<std::pair<float, float>, 4> expected = {
        std::pair<float, float>(-3.0f, -3.0f), {3.0f, -3.0f},
        {3.0f, 3.0f}, {-3.0f, 3.0f}};
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_FLOAT_EQ(contours.front()[i].x, expected[i].first);
        EXPECT_FLOAT_EQ(contours.front()[i].y, expected[i].second);
    }
}
