// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <limits>

#include "far_planner/graph_node_index.h"

namespace {
NavNodePtr Node(std::size_t id, float x) {
    auto node = std::make_shared<NavNode>();
    node->id = id;
    node->position = Point3D(x, 0.0f, 0.0f);
    node->is_odom = false;
    node->is_goal = false;
    node->is_navpoint = true;
    return node;
}
}

TEST(GraphNodeIndex, SkipsNonsearchableNodesWithoutGhostPoints) {
    auto odom = Node(1, 0.0f);
    odom->is_odom = true;
    auto outside_goal = Node(2, 1.0f);
    outside_goal->is_goal = true;
    outside_goal->is_navpoint = false;
    auto graph_node = Node(3, 10.0f);

    GraphNodeIndex index;
    index.Build({odom, outside_goal, graph_node});
    EXPECT_EQ(index.Size(), 1U);
    EXPECT_EQ(index.Nearest(Point3D(0.0f, 0.0f, 0.0f), 0.5f), nullptr);
    EXPECT_EQ(index.Nearest(Point3D(10.0f, 0.0f, 0.0f), 0.5f), graph_node);

    index.Build({});
    EXPECT_EQ(index.Size(), 0U);
    EXPECT_EQ(index.Nearest(Point3D(0.0f, 0.0f, 0.0f), 1.0f), nullptr);
}

TEST(GraphNodeIndex, UsesLinearRadiusAndPreservesFullNodeId) {
    auto node = Node(16777217U, 0.0f); // Not exactly representable as float intensity.
    GraphNodeIndex index;
    index.Build({node});
    EXPECT_EQ(index.Nearest(Point3D(1.5f, 0.0f, 0.0f), 2.0f), node);
    EXPECT_EQ(index.Nearest(Point3D(1.5f, 0.0f, 0.0f), 1.4f), nullptr);
    EXPECT_EQ(index.Nearest(Point3D(0.0f, 0.0f, 0.0f),
                            std::numeric_limits<float>::quiet_NaN()), nullptr);
}
