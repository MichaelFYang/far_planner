// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <limits>

#include "far_planner/graph_msger.h"

namespace {
#ifdef FAR_PLANNER_ROS1
using GraphMessage = visibility_graph_msg::Graph;
using NodeMessage = visibility_graph_msg::Node;
#else
using GraphMessage = visibility_graph_msg::msg::Graph;
using NodeMessage = visibility_graph_msg::msg::Node;
#endif

GraphMessage ValidGraph() {
    GraphMessage graph;
    graph.header.frame_id = "map";
    NodeMessage node;
    node.id = 7;
#ifdef FAR_PLANNER_ROS1
    node.FreeType = 1;
#else
    node.freetype = 1;
#endif
    node.position.x = 1.0;
    node.surface_dirs.resize(2);
    graph.nodes.push_back(node);
    graph.size = 1;
    return graph;
}
}

TEST(GraphMessage, AcceptsWellFormedWorldFrameGraph) {
    auto graph = ValidGraph();
    std::string error;
    EXPECT_TRUE(GraphMsger::ValidateGraphMessage(graph, "/map", error));
    EXPECT_TRUE(error.empty());
}

TEST(GraphMessage, RejectsBadFrameGeometryAndMissingDirections) {
    auto graph = ValidGraph();
    std::string error;
    graph.header.frame_id = "odom";
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    graph = ValidGraph();
    graph.nodes[0].surface_dirs.clear();
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    graph = ValidGraph();
    graph.nodes[0].position.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
}

TEST(GraphMessage, RejectsDuplicateIdsAndMissingConnectionTargets) {
    auto graph = ValidGraph();
    std::string error;
    graph.nodes.push_back(graph.nodes[0]);
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    graph = ValidGraph();
    graph.nodes[0].connect_nodes.push_back(99);
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    graph = ValidGraph();
    graph.nodes[0].connect_nodes.push_back(7);
    EXPECT_TRUE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    EXPECT_TRUE(error.empty());
}
