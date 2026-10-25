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
