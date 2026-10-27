// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>

#ifdef FAR_PLANNER_ROS1
#include <visibility_graph_msg/Graph.h>
#include <visibility_graph_msg/Node.h>
using GraphMessage = visibility_graph_msg::Graph;
using NodeMessage = visibility_graph_msg::Node;
#else
#include <visibility_graph_msg/msg/graph.hpp>
#include <visibility_graph_msg/msg/node.hpp>
using GraphMessage = visibility_graph_msg::msg::Graph;
using NodeMessage = visibility_graph_msg::msg::Node;
#endif

#include <visibility_graph_msg/graph_validation.h>

namespace {

GraphMessage ValidGraph() {
    GraphMessage graph;
    graph.header.frame_id = "map";
    for (unsigned int id : {7u, 9u}) {
        NodeMessage node;
        node.header.frame_id = "/map";
        node.id = id;
#ifdef FAR_PLANNER_ROS1
        node.FreeType = 1;
#else
        node.freetype = 1;
#endif
        node.position.x = static_cast<double>(id);
        node.surface_dirs.resize(2);
        graph.nodes.push_back(node);
    }
    graph.nodes[0].connect_nodes.push_back(9);
    graph.size = static_cast<std::uint32_t>(graph.nodes.size());
    return graph;
}

bool Validate(const GraphMessage& graph, std::string& error) {
    return graph_message_validation::ValidateGraph(
        graph, "/map", [](const auto& node) {
#ifdef FAR_PLANNER_ROS1
            return node.FreeType;
#else
            return node.freetype;
#endif
        }, error);
}

TEST(GraphMessageValidation, RejectsUnknownConnectionTargets) {
    auto graph = ValidGraph();
    std::string error;
    EXPECT_TRUE(Validate(graph, error));
    EXPECT_TRUE(error.empty());
    graph.nodes[0].trajectory_connects.push_back(99);
    EXPECT_FALSE(Validate(graph, error));
    EXPECT_NE(error.find("absent node"), std::string::npos);
    graph = ValidGraph();
    graph.nodes[1].id = 7;
    EXPECT_FALSE(Validate(graph, error));
}

TEST(GraphMessageValidation, RejectsBadGeometryAndAcceptsEmptyClear) {
    auto graph = ValidGraph();
    std::string error;
    graph.nodes[0].position.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(Validate(graph, error));
    graph = ValidGraph();
    graph.nodes[0].header.frame_id = "odom";
    EXPECT_FALSE(Validate(graph, error));
    graph = ValidGraph();
    graph.nodes.clear();
    graph.size = 0;
    EXPECT_TRUE(Validate(graph, error));
}

}  // namespace

#ifdef FAR_PLANNER_ROS1
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif
