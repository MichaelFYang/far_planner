// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "far_planner/graph_msger.h"
#include "far_planner/parameter_validation.h"

namespace {
#ifdef FAR_PLANNER_ROS1
using GraphMessage = visibility_graph_msg::Graph;
using NodeMessage = visibility_graph_msg::Node;
#else
using GraphMessage = visibility_graph_msg::msg::Graph;
using NodeMessage = visibility_graph_msg::msg::Node;
#endif

NavNodePtr EncodeNode(std::size_t id) {
    auto node = std::make_shared<NavNode>();
    node->id = id;
    node->position = Point3D(static_cast<float>(id), 0.0f, 0.0f);
    node->surf_dirs = {Point3D(1.0f, 0.0f, 0.0f), Point3D(0.0f, 1.0f, 0.0f)};
    node->free_direct = CONVEX;
    node->is_finalized = true;
    node->is_navpoint = true;
    return node;
}

}

class GraphEncodingTestAccess {
public:
    static bool TryEncode(const NodePtrStack& nodes, GraphMessage& graph) {
        graph.header.frame_id = "map";
        return GraphMsger::EncodeGraph(nodes, graph);
    }

    static GraphMessage Encode(const NodePtrStack& nodes) {
        GraphMessage graph;
        EXPECT_TRUE(TryEncode(nodes, graph));
        return graph;
    }
};

namespace {

GraphMessage ValidGraph() {
    GraphMessage graph;
    graph.header.frame_id = "map";
    NodeMessage node;
    node.header.frame_id = "map";
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

TEST(GraphMessage, RejectsInconsistentNodeFrameAndCount) {
    auto graph = ValidGraph();
    std::string error;
    graph.nodes[0].header.frame_id = "odom";
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    graph = ValidGraph();
    graph.nodes[0].header.frame_id.clear();
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    graph = ValidGraph();
    graph.nodes[0].header.frame_id = "/map";
    EXPECT_TRUE(GraphMsger::ValidateGraphMessage(graph, "map", error));
    graph.size = 0;
    EXPECT_FALSE(GraphMsger::ValidateGraphMessage(graph, "map", error));
}

TEST(GraphMessage, EncodesOnlyEdgesToPublishedNodes) {
    const auto kept = EncodeNode(1);
    const auto no_regular_edges = EncodeNode(2);
    const auto connected = EncodeNode(3);
    const auto odom = EncodeNode(4);
    const auto absent = EncodeNode(5);
    odom->is_odom = true;
    const NodePtrStack neighbors = {no_regular_edges, connected, odom, absent};
    kept->connect_nodes = neighbors;
    kept->poly_connects = neighbors;
    kept->contour_connects = neighbors;
    kept->trajectory_connects = neighbors;
    connected->connect_nodes = {no_regular_edges};

    const auto graph = GraphEncodingTestAccess::Encode(
        {kept, no_regular_edges, connected, odom});
    ASSERT_EQ(graph.size, 2U);
    ASSERT_EQ(graph.nodes.size(), 2U);
    EXPECT_EQ(graph.nodes[0].id, 1U);
    EXPECT_EQ(graph.nodes[1].id, 3U);
    EXPECT_EQ(graph.nodes[0].connect_nodes, std::vector<std::uint32_t>({3}));
    EXPECT_EQ(graph.nodes[0].poly_connects, std::vector<std::uint32_t>({3}));
    EXPECT_EQ(graph.nodes[0].contour_connects, std::vector<std::uint32_t>({3}));
    EXPECT_EQ(graph.nodes[0].trajectory_connects, std::vector<std::uint32_t>({3}));
    EXPECT_TRUE(graph.nodes[1].connect_nodes.empty());
    std::string error;
    EXPECT_TRUE(GraphMsger::ValidateGraphMessage(graph, "map", error)) << error;
}

TEST(GraphMessage, RetainsNodesWithOnlyNonVisibilityEdges) {
    const auto trajectory_a = EncodeNode(11);
    const auto trajectory_b = EncodeNode(12);
    const auto contour_a = EncodeNode(21);
    const auto contour_b = EncodeNode(22);
    const auto polygon_a = EncodeNode(31);
    const auto polygon_b = EncodeNode(32);
    trajectory_a->trajectory_connects = {trajectory_b};
    trajectory_b->trajectory_connects = {trajectory_a};
    contour_a->contour_connects = {contour_b};
    contour_b->contour_connects = {contour_a};
    polygon_a->poly_connects = {polygon_b};
    polygon_b->poly_connects = {polygon_a};

    const auto graph = GraphEncodingTestAccess::Encode(
        {trajectory_a, trajectory_b, contour_a, contour_b, polygon_a, polygon_b});
    trajectory_a->trajectory_connects.clear();
    trajectory_b->trajectory_connects.clear();
    contour_a->contour_connects.clear();
    contour_b->contour_connects.clear();
    polygon_a->poly_connects.clear();
    polygon_b->poly_connects.clear();

    ASSERT_EQ(graph.nodes.size(), 6U);
    EXPECT_EQ(graph.size, 6U);
    EXPECT_EQ(graph.nodes[0].trajectory_connects, std::vector<std::uint32_t>({12}));
    EXPECT_EQ(graph.nodes[1].trajectory_connects, std::vector<std::uint32_t>({11}));
    EXPECT_EQ(graph.nodes[2].contour_connects, std::vector<std::uint32_t>({22}));
    EXPECT_EQ(graph.nodes[3].contour_connects, std::vector<std::uint32_t>({21}));
    EXPECT_EQ(graph.nodes[4].poly_connects, std::vector<std::uint32_t>({32}));
    EXPECT_EQ(graph.nodes[5].poly_connects, std::vector<std::uint32_t>({31}));
    std::string error;
    EXPECT_TRUE(GraphMsger::ValidateGraphMessage(graph, "map", error)) << error;
}

TEST(GraphMessage, RejectsUnrepresentableAndDuplicateNodeIds) {
    const std::size_t largest_id = std::numeric_limits<std::uint32_t>::max();
    const auto largest = EncodeNode(largest_id);
    const auto neighbor = EncodeNode(7);
    largest->connect_nodes = {neighbor};
    neighbor->connect_nodes = {largest};
    auto graph = GraphEncodingTestAccess::Encode({largest, neighbor});
    largest->connect_nodes.clear();
    neighbor->connect_nodes.clear();
    ASSERT_EQ(graph.nodes.size(), 2U);
    EXPECT_EQ(graph.nodes[0].id, largest_id);

    if (std::numeric_limits<std::size_t>::max() > largest_id) {
        const auto oversized = EncodeNode(largest_id + 1U);
        oversized->connect_nodes = {neighbor};
        neighbor->connect_nodes = {oversized};
        EXPECT_FALSE(GraphEncodingTestAccess::TryEncode({oversized, neighbor}, graph));
        oversized->connect_nodes.clear();
        neighbor->connect_nodes.clear();
        EXPECT_TRUE(graph.nodes.empty());
        EXPECT_EQ(graph.size, 0U);
    }

    const auto duplicate = EncodeNode(neighbor->id);
    neighbor->connect_nodes = {largest};
    duplicate->connect_nodes = {largest};
    EXPECT_FALSE(GraphEncodingTestAccess::TryEncode({neighbor, duplicate}, graph));
    neighbor->connect_nodes.clear();
    duplicate->connect_nodes.clear();
    EXPECT_TRUE(graph.nodes.empty());
    EXPECT_EQ(graph.size, 0U);
}

TEST(GraphMessage, RejectsRobotIdsOutsideWireRange) {
    EXPECT_EQ(far_planner::CheckedRobotId(0), 0U);
    EXPECT_EQ(far_planner::CheckedRobotId(65535), 65535U);
    EXPECT_THROW(far_planner::CheckedRobotId(-1), std::invalid_argument);
    EXPECT_THROW(far_planner::CheckedRobotId(65536), std::invalid_argument);
}
