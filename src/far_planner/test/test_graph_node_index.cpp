// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <limits>
#include <random>

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

TEST(GraphNodeIndex, FiltersNeighborsAndKeepsTheStrictRadiusBoundary) {
    auto wrong_type = Node(1, 0.0f);
    auto farther = Node(2, 0.5f);
    auto nearer = Node(3, 0.25f);
    farther->is_boundary = nearer->is_boundary = true;
    GraphNodeIndex index;
    index.Build({wrong_type, farther, nearer});
    const auto boundary = [](const NavNodePtr& node) { return node->is_boundary; };
    const Point3D query(0.0f, 0.0f, 0.0f);
    EXPECT_EQ(index.Nearest(query, 0.5f, boundary), nearer);
    EXPECT_EQ(index.Nearest(query, 0.25f, boundary), nullptr);
    EXPECT_EQ(index.Nearest(query, 1.0f, [](const NavNodePtr&) { return false; }), nullptr);
    EXPECT_EQ(index.Nearest(query, 1.0f), wrong_type);
}

TEST(GraphNodeIndex, FilteredQueriesMatchIndependentLinearSearch) {
    std::mt19937 random(7341);
    std::uniform_real_distribution<float> coordinate(-2.0f, 2.0f);
    std::uniform_real_distribution<float> radius(0.1f, 3.0f);
    NodePtrStack graph;
    for (std::size_t i = 0; i < 64; ++i) {
        auto node = Node(i, coordinate(random));
        node->position.y = coordinate(random);
        node->position.z = coordinate(random);
        node->is_navpoint = (i & 1U) != 0;
        node->is_boundary = (i & 2U) != 0;
        graph.push_back(node);
    }
    GraphNodeIndex index;
    index.Build(graph);
    for (std::size_t i = 0; i < 100; ++i) {
        const Point3D query(coordinate(random), coordinate(random), coordinate(random));
        const float limit = radius(random);
        const auto matches = [i](const NavNodePtr& node) {
            return node->is_navpoint == ((i & 1U) != 0) &&
                   node->is_boundary == ((i & 2U) != 0);
        };
        NavNodePtr expected;
        float best_squared_distance = limit * limit;
        for (const auto& node : graph) {
            if (!matches(node)) continue;
            const auto delta = node->position - query;
            const float distance = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
            if (distance < best_squared_distance) {
                best_squared_distance = distance;
                expected = node;
            }
        }
        EXPECT_EQ(index.Nearest(query, limit, matches), expected) << "query " << i;
    }
}
