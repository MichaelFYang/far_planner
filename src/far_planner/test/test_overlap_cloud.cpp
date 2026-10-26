// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <initializer_list>

#include "far_planner/utility.h"

namespace {

struct RestoreOverlapSettings {
    float leaf = FARUtil::kLeafSize;
    float new_point_threshold = FARUtil::kNewPIThred;
    ~RestoreOverlapSettings() {
        FARUtil::kLeafSize = leaf;
        FARUtil::kNewPIThred = new_point_threshold;
    }
};

PCLPoint Point(float x, float intensity) {
    PCLPoint point{};
    point.x = x;
    point.intensity = intensity;
    return point;
}

TEST(OverlapCloud, TagsTemporaryCloudWithoutChangingInputs) {
    const RestoreOverlapSettings restore;
    FARUtil::kLeafSize = 1.0f;
    FARUtil::kNewPIThred = 2.0f;
    PointCloudPtr current(new PointCloud);
    current->push_back(Point(0.0f, 7.0f));
    current->push_back(Point(3.0f, 9.0f));
    PointCloudPtr reference(new PointCloud);
    reference->push_back(Point(0.0f, 42.0f));

    PointCloudPtr fresh(new PointCloud);
    FARUtil::ExtractNewObsPointCloud(current, reference, fresh);
    ASSERT_EQ(fresh->size(), 1U);
    EXPECT_FLOAT_EQ(fresh->front().x, 3.0f);
    EXPECT_EQ(fresh->width, fresh->size());
    EXPECT_EQ(fresh->height, 1U);
    EXPECT_FLOAT_EQ(current->points[0].intensity, 7.0f);
    EXPECT_FLOAT_EQ(current->points[1].intensity, 9.0f);
    EXPECT_FLOAT_EQ(reference->front().intensity, 42.0f);

    PointCloudPtr overlap(new PointCloud);
    FARUtil::ExtractOverlapCloud(current, reference, overlap);
    ASSERT_EQ(overlap->size(), 1U);
    EXPECT_FLOAT_EQ(overlap->front().x, 0.0f);
    EXPECT_FLOAT_EQ(current->points[0].intensity, 7.0f);
    EXPECT_FLOAT_EQ(reference->front().intensity, 42.0f);

    for (const bool legacy_copy_flag : {false, true}) {
        PointCloudPtr removable(new PointCloud(*current));
        FARUtil::RemoveOverlapCloud(removable, reference, legacy_copy_flag);
        ASSERT_EQ(removable->size(), 1U);
        EXPECT_FLOAT_EQ(removable->front().x, 3.0f);
        EXPECT_FLOAT_EQ(reference->front().intensity, 42.0f);
    }
}

}  // namespace
