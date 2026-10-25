// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "far_planner/dynamic_graph.h"

TEST(GraphLifecycle, EraseNodeFromStackRemovesFirstLastAndConsecutiveCopies)
{
    auto first = std::make_shared<NavNode>();
    auto middle = std::make_shared<NavNode>();
    auto last = std::make_shared<NavNode>();
    NodePtrStack nodes = {first, middle, middle, last, first};
    FARUtil::EraseNodeFromStack(first, nodes);
    EXPECT_EQ(nodes, (NodePtrStack{middle, middle, last}));
    FARUtil::EraseNodeFromStack(middle, nodes);
    EXPECT_EQ(nodes, (NodePtrStack{last}));
    FARUtil::EraseNodeFromStack(last, nodes);
    EXPECT_TRUE(nodes.empty());
}

TEST(GraphLifecycle, ResetReleasesReciprocalEdgesAndSearchParents)
{
    DynamicGraph graph;
    graph.ResetCurrentGraph();
    auto a = std::make_shared<NavNode>();
    auto b = std::make_shared<NavNode>();
    a->id = 1;
    b->id = 2;
    DynamicGraph::AddNodeToGraph(a);
    DynamicGraph::AddNodeToGraph(b);
    DynamicGraph::AddEdge(a, b);
    DynamicGraph::AddPolyEdge(a, b);
    a->parent = b;
    b->free_parent = a;
    std::weak_ptr<NavNode> weak_a = a, weak_b = b;
    graph.ResetCurrentGraph();
    a.reset();
    b.reset();
    const bool released = weak_a.expired() && weak_b.expired();
    if (!released) {
        // Keep a failing baseline run from leaking its fixture allocations.
        a = weak_a.lock();
        b = weak_b.lock();
        a->connect_nodes.clear(); a->poly_connects.clear(); a->parent.reset();
        b->connect_nodes.clear(); b->poly_connects.clear(); b->free_parent.reset();
        a.reset(); b.reset();
    }
    EXPECT_TRUE(released);
}

TEST(GraphLifecycle, ContourResetReleasesLinkedVertices)
{
    ContourGraph graph;
    auto a = std::make_shared<CTNode>();
    auto b = std::make_shared<CTNode>();
    a->front = b;
    b->back = a;
    ContourGraph::contour_graph_ = {a, b};
    std::weak_ptr<CTNode> weak_a = a, weak_b = b;
    graph.ResetCurrentContour();
    a.reset(); b.reset();
    const bool released = weak_a.expired() && weak_b.expired();
    if (!released) {
        a = weak_a.lock(); b = weak_b.lock();
        a->front.reset(); b->back.reset();
        a.reset(); b.reset();
    }
    EXPECT_TRUE(released);
}

TEST(GraphLifecycle, OrdinaryContourUpdateReleasesRetiredVertices)
{
    ContourGraph graph;
    graph.ResetCurrentContour();
    auto a = std::make_shared<CTNode>();
    auto b = std::make_shared<CTNode>();
    a->front = b;
    b->back = a;
    a->connect_nodes.push_back(b);
    b->connect_nodes.push_back(a);
    ContourGraph::contour_graph_ = {a, b};

    auto nav = std::make_shared<NavNode>();
    nav->ctnode = a; // Navigation may retain the old contour until rematching.
    auto odom = std::make_shared<NavNode>();
    odom->position = Point3D(0.0f, 0.0f, 0.0f);
    std::weak_ptr<CTNode> weak_a = a, weak_b = b;

    graph.UpdateContourGraph(odom, {});
    a.reset();
    b.reset();
    const bool neighbor_released = weak_b.expired();
    nav->ctnode.reset();
    const bool all_released = weak_a.expired() && weak_b.expired();
    if (!all_released) {
        a = weak_a.lock(); b = weak_b.lock();
        if (a) { a->front.reset(); a->back.reset(); a->connect_nodes.clear(); }
        if (b) { b->front.reset(); b->back.reset(); b->connect_nodes.clear(); }
        a.reset(); b.reset();
    }
    EXPECT_TRUE(neighbor_released);
    EXPECT_TRUE(all_released);
    graph.ResetCurrentContour();
}

TEST(GraphLifecycle, PillarThresholdUsesClosedPolygonPerimeter)
{
    ContourGraph graph;
    ContourGraphParams params;
    params.kPillarPerimeter = 10.0f;
    graph.Init(nullptr, params);
    auto odom = std::make_shared<NavNode>();
    odom->position = Point3D(-10.0f, -10.0f, 0.0f);

    // The open chain is 7 m, but the triangle's full perimeter is 12 m.
    graph.UpdateContourGraph(odom, {{Point3D(0.0f, 0.0f, 0.0f),
                                     Point3D(3.0f, 0.0f, 0.0f),
                                     Point3D(3.0f, 4.0f, 0.0f)}});
    ASSERT_EQ(ContourGraph::contour_graph_.size(), 3U);
    EXPECT_FALSE(ContourGraph::contour_graph_.front()->poly_ptr->is_pillar);
    EXPECT_FLOAT_EQ(ContourGraph::contour_graph_.front()->poly_ptr->perimeter, 12.0f);
    graph.ResetCurrentContour();
}
