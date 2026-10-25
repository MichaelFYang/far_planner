// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>
#include <random>

#include "far_planner/graph_planner.h"
#include "far_planner/terrain_planner.h"

namespace {
NavNodePtr Node(std::size_t id, float x, float y)
{
    auto node = std::make_shared<NavNode>();
    node->id = id;
    node->position = Point3D(x, y, 0.0f);
    node->is_boundary = false;
    node->is_covered = true;
    return node;
}

void Link(const NavNodePtr& a, const NavNodePtr& b)
{
    a->connect_nodes.push_back(b);
    b->connect_nodes.push_back(a);
}

TEST(GraphSearch, DecreaseOfQueuedNodeMustReorderBothTraversals)
{
    // The queue initially contains node 1 via 4. Later 3 improves its cost,
    // but a queue ordered by mutable NavNode::gscore can still pop it early.
    NodePtrStack graph = {Node(0, 24, 29), Node(1, 24, 14),
                          Node(2, 18, 2), Node(3, 24, 16), Node(4, 4, 14)};
    for (const auto& edge : {std::pair<int, int>(0, 2), {0, 4}, {1, 3},
                             {1, 4}, {2, 3}, {2, 4}, {3, 4}}) {
        Link(graph[edge.first], graph[edge.second]);
    }
    GraphPlanner planner;
    planner.UpdaetVGraph(graph);
    planner.UpdateGraphTraverability(graph[0], nullptr);
    const float expected = (graph[0]->position - graph[2]->position).norm() +
                           (graph[2]->position - graph[3]->position).norm() +
                           (graph[3]->position - graph[1]->position).norm();
    EXPECT_NEAR(graph[1]->gscore, expected, 1e-4f);
    EXPECT_NEAR(graph[1]->fgscore, expected, 1e-4f);
    EXPECT_EQ(graph[1]->parent, graph[3]);
    EXPECT_EQ(graph[1]->free_parent, graph[3]);
    for (auto& node : graph) {
        node->connect_nodes.clear();
        node->parent.reset();
        node->free_parent.reset();
    }
}
TEST(GraphSearch, MatchesIndependentShortestPathsForGeneralAndCoveredGraph)
{
    std::mt19937 generator(271828);
    GraphPlanner planner;
    for (int iteration = 0; iteration < 40; ++iteration) {
        NodePtrStack graph;
        for (int i = 0; i < 12; ++i) {
            graph.push_back(Node(i, static_cast<float>(generator() % 50),
                                    static_cast<float>(generator() % 50)));
            graph.back()->is_covered = i == 0 || generator() % 4 != 0;
        }
        for (std::size_t i = 0; i < graph.size(); ++i) {
            for (std::size_t j = i + 1; j < graph.size(); ++j) {
                if (generator() % 4 == 0) Link(graph[i], graph[j]);
            }
        }
        planner.UpdaetVGraph(graph);
        planner.UpdateGraphTraverability(graph[0], nullptr);
        for (bool covered_only : {false, true}) {
            // O(V^2) reference, deliberately independent of a heap.
            std::vector<float> distance(graph.size(), std::numeric_limits<float>::infinity());
            std::vector<bool> visited(graph.size(), false);
            distance[0] = 0.0f;
            for (std::size_t step = 0; step < graph.size(); ++step) {
                std::size_t current = graph.size();
                for (std::size_t i = 0; i < graph.size(); ++i) {
                    if (!visited[i] && (current == graph.size() || distance[i] < distance[current])) {
                        current = i;
                    }
                }
                if (current == graph.size() || !std::isfinite(distance[current])) break;
                visited[current] = true;
                for (const auto& neighbor : graph[current]->connect_nodes) {
                    if (covered_only && !neighbor->is_covered) continue;
                    const float candidate = distance[current] +
                        (graph[current]->position - neighbor->position).norm();
                    distance[neighbor->id] = std::min(distance[neighbor->id], candidate);
                }
            }
            for (std::size_t i = 0; i < graph.size(); ++i) {
                const float actual = covered_only ? graph[i]->fgscore : graph[i]->gscore;
                const bool traversable = covered_only ? graph[i]->is_free_traversable : graph[i]->is_traversable;
                if (std::isfinite(distance[i])) {
                    EXPECT_NEAR(actual, distance[i], 1e-3f) << iteration << " node " << i;
                    EXPECT_TRUE(traversable) << iteration << " node " << i;
                } else {
                    EXPECT_EQ(actual, FARUtil::kINF) << iteration << " node " << i;
                    EXPECT_FALSE(traversable) << iteration << " node " << i;
                }
            }
        }
        for (auto& node : graph) {
            node->connect_nodes.clear();
            node->parent.reset();
            node->free_parent.reset();
        }
    }
}

TEST(TerrainSearch, QueueDecreaseMustReturnTheShortestGridPath)
{
#ifdef FAR_PLANNER_ROS1
    int argc = 0;
    char** argv = nullptr;
    if (!ros::isInitialized()) ros::init(argc, argv, "terrain_search_test");
    ros::NodeHandle node;
#else
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("terrain_search_test");
#endif
    const float previous_range = FARUtil::kLocalPlanRange;
    FARUtil::kLocalPlanRange = 0.0f;  // A five by five test grid.
    TerrainPlannerParams params;
    params.world_frame = "map";
    params.radius = 2.5f;
    params.voxel_size = 1.0f;
    params.inflate_size = 0;
    TerrainPlanner planner;
    planner.Init(node, params);
    planner.UpdateCenterNode(Node(99, 0.0f, 0.0f));
    PointCloudPtr obstacles(new PointCloud);
    for (int index : {0, 10, 13, 18}) {
        PCLPoint point{};
        point.x = static_cast<float>(index % 5 - 2);
        point.y = static_cast<float>(index / 5 - 2);
        point.z = 0.0f;
        obstacles->push_back(point);
    }
    planner.SetLocalTerrainObsCloud(obstacles);
    PointStack path;
    const bool found = planner.PlanPathFromPToP(Point3D(-2.0f, -1.0f, 0.0f),
                                                Point3D(2.0f, 1.0f, 0.0f), path);
    FARUtil::kLocalPlanRange = previous_range;
    ASSERT_TRUE(found);
    float length = 0.0f;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    EXPECT_NEAR(length, 4.0f + std::sqrt(2.0f), 1e-4f);
}
}  // namespace
