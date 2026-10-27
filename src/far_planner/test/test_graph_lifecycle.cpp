// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include "far_planner/dynamic_graph.h"
#include "far_planner/far_planner.h"
#include "far_planner/vote_selection.h"

class GraphVoteTestAccess {
public:
    static void Fill(DynamicGraph& graph, const NavNodePtr& first,
                     const NavNodePtr& second, int count) {
        graph.FillContourConnect(first, second, count);
    }

    static void Disconnect(const NavNodePtr& first, const NavNodePtr& second) {
        DynamicGraph::DeleteContourConnect(first, second);
    }

    static void Connect(const NavNodePtr& first, const NavNodePtr& second) {
        DynamicGraph::AddContourConnect(first, second);
        DynamicGraph::AddEdge(first, second);
    }

    static void Select(DynamicGraph& graph, const NavNodePtr& node) {
#ifndef FAR_PLANNER_ROS1
        graph.nh_ = rclcpp::Node::make_shared("graph_vote_test");
#endif
        graph.TopTwoContourConnector(node);
    }

    static bool NeedsInterNav(DynamicGraph& graph, const NavNodePtr& odom,
                              const NavNodePtr& internav) {
        graph.odom_node_ptr_ = odom;
        graph.cur_internav_ptr_ = internav;
        graph.is_bridge_internav_ = true;
        graph.last_connect_pos_ = Point3D(0.0f, 0.0f, 0.0f);
        return graph.IsInterNavpointNecessary();
    }

    static void ClearMerged(DynamicGraph& graph) {
        graph.ClearMergedNodesInGraph();
    }

    static void SetTrajectoryState(DynamicGraph& graph, const NavNodePtr& odom,
                                   const NavNodePtr& current, const NodePtrStack& nearby,
                                   const Point3D& last_position, bool bridge) {
        graph.odom_node_ptr_ = odom;
        graph.cur_internav_ptr_ = current;
        graph.internav_near_nodes_ = nearby;
        graph.last_connect_pos_ = last_position;
        graph.is_bridge_internav_ = bridge;
    }
};

TEST(GraphNodes, DirectDefaultConstructionHasNeutralState)
{
    Polygon polygon;
    EXPECT_EQ(polygon.N, 0U);
    EXPECT_FALSE(polygon.is_robot_inside);
    EXPECT_FALSE(polygon.is_pillar);
    EXPECT_FLOAT_EQ(polygon.perimeter, 0.0f);

    CTNode contour;
    EXPECT_FALSE(contour.is_global_match);
    EXPECT_EQ(contour.free_direct, NodeFreeDirect::UNKNOW);
    EXPECT_EQ(contour.front, nullptr);
    EXPECT_EQ(contour.back, nullptr);

    NavNode nav;
    EXPECT_EQ(nav.id, 0U);
    EXPECT_FALSE(nav.is_active);
    EXPECT_FALSE(nav.is_goal);
    EXPECT_EQ(nav.node_type, NodeType::NOT_DEFINED);
    EXPECT_EQ(nav.free_direct, NodeFreeDirect::UNKNOW);
    EXPECT_EQ(nav.parent, nullptr);
    EXPECT_TRUE(nav.connect_nodes.empty());
}

TEST(GraphVotes, TopTwoSelectionPreservesStrictRankAndTies)
{
    far_planner::TopTwoScores empty;
    EXPECT_FALSE(empty.Includes(0));
    std::mt19937 random(12345);
    std::uniform_int_distribution<int> score(-5, 5);
    for (int count = 1; count <= 32; ++count) {
        for (int sample = 0; sample < 40; ++sample) {
            far_planner::TopTwoScores top;
            std::vector<int> scores;
            for (int i = 0; i < count; ++i) {
                const int value = score(random);
                top.Observe(value);
                scores.push_back(value);
            }
            std::sort(scores.begin(), scores.end(), std::greater<int>());
            for (const int value : scores) {
                const int rank = std::count_if(scores.begin(), scores.end(),
                    [value](int other) { return other > value; });
                EXPECT_EQ(top.Includes(value), rank < 2);
            }
        }
    }
    far_planner::TopTwoScores extremes;
    extremes.Observe(std::numeric_limits<int>::min());
    extremes.Observe(std::numeric_limits<int>::max());
    EXPECT_TRUE(extremes.Includes(std::numeric_limits<int>::min()));
}

TEST(GraphVotes, MissingPotentialVoteDoesNotDereferenceTheMapEnd)
{
#ifdef FAR_PLANNER_ROS1
    int argc = 0;
    char** argv = nullptr;
    if (!ros::isInitialized()) ros::init(argc, argv, "graph_vote_test");
#else
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
#endif
    DynamicGraph graph;
    auto node = std::make_shared<NavNode>();
    auto potential = std::make_shared<NavNode>();
    potential->id = 42;
    node->potential_contours.push_back(potential);
    GraphVoteTestAccess::Connect(node, potential);
    ASSERT_EQ(node->contour_connects.size(), 1U);
    ASSERT_EQ(node->connect_nodes.size(), 1U);
    GraphVoteTestAccess::Select(graph, node);
    EXPECT_TRUE(node->contour_connects.empty());
    EXPECT_TRUE(node->connect_nodes.empty());
    EXPECT_TRUE(potential->contour_connects.empty());
    EXPECT_TRUE(potential->connect_nodes.empty());
#ifndef FAR_PLANNER_ROS1
    rclcpp::shutdown();
#endif
}

TEST(GraphVotes, RepeatedContourFillPreservesVotesAndLinks)
{
    DynamicGraph graph;
    auto first = std::make_shared<NavNode>();
    auto second = std::make_shared<NavNode>();
    first->id = 51;
    second->id = 52;
    GraphVoteTestAccess::Fill(graph, first, second, 4);
    ASSERT_EQ(first->contour_votes.count(second->id), 1U);
    ASSERT_EQ(second->contour_votes.count(first->id), 1U);
    first->contour_votes.at(second->id).back() = 0;
    second->contour_votes.at(first->id).back() = 0;
    GraphVoteTestAccess::Fill(graph, first, second, 8);
    const std::deque<int> expected{1, 1, 1, 0};
    EXPECT_EQ(first->contour_votes.at(second->id), expected);
    EXPECT_EQ(second->contour_votes.at(first->id), expected);
    EXPECT_EQ(first->potential_contours.size(), 1U);
    EXPECT_EQ(second->potential_contours.size(), 1U);
    EXPECT_EQ(first->contour_connects.size(), 1U);
    EXPECT_EQ(second->contour_connects.size(), 1U);
    GraphVoteTestAccess::Disconnect(first, second);
    first->potential_contours.clear();
    second->potential_contours.clear();
}

TEST(GraphVotes, EmptyOdomVoteHistoryDoesNotAccessItsLastElement)
{
    struct RestoreGlobals {
        Point3D free_odom = FARUtil::free_odom_p;
        float near_dist = FARUtil::kNearDist;
        ~RestoreGlobals() {
            FARUtil::free_odom_p = free_odom;
            FARUtil::kNearDist = near_dist;
        }
    } restore;
    FARUtil::free_odom_p = Point3D(0.0f, 0.0f, 0.0f);
    FARUtil::kNearDist = 1.0f;

    DynamicGraph graph;
    auto odom = std::make_shared<NavNode>();
    auto internav = std::make_shared<NavNode>();
    internav->id = 42;
    odom->edge_votes.emplace(internav->id, std::deque<int>{});
    EXPECT_FALSE(GraphVoteTestAccess::NeedsInterNav(graph, odom, internav));
}

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

TEST(GraphLifecycle, PlannerDestructionReleasesLinkedGraphsWithoutExplicitReset)
{
#ifdef FAR_PLANNER_ROS1
    int argc = 0;
    char** argv = nullptr;
    if (!ros::isInitialized()) ros::init(argc, argv, "planner_shutdown_test");
#else
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
#endif
    DynamicGraph graph;
    ContourGraph contours;
    graph.ResetCurrentGraph();
    contours.ResetCurrentContour();
    std::weak_ptr<NavNode> weak_nav;
    std::weak_ptr<CTNode> weak_contour;
    {
        FARMaster planner;
#ifndef FAR_PLANNER_ROS1
        struct ShutdownBeforePlanner {
            ~ShutdownBeforePlanner() { rclcpp::shutdown(); }
        } shutdown_before_planner;
#endif
        auto first = std::make_shared<NavNode>();
        auto second = std::make_shared<NavNode>();
        first->id = 1;
        second->id = 2;
        DynamicGraph::AddNodeToGraph(first);
        DynamicGraph::AddNodeToGraph(second);
        DynamicGraph::AddEdge(first, second);
        first->parent = second;
        second->free_parent = first;
        auto front = std::make_shared<CTNode>();
        auto back = std::make_shared<CTNode>();
        front->front = back;
        back->back = front;
        first->ctnode = front;
        ContourGraph::contour_graph_ = {front, back};
        weak_nav = first;
        weak_contour = front;
    }
    EXPECT_TRUE(DynamicGraph::GetNavGraph().empty());
    EXPECT_TRUE(ContourGraph::contour_graph_.empty());
    EXPECT_TRUE(weak_nav.expired());
    EXPECT_TRUE(weak_contour.expired());
    // Also release fixture state when running against the failing baseline.
    graph.ResetCurrentGraph();
    contours.ResetCurrentContour();
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
        if (a) { a->front.reset(); a->back.reset(); }
        if (b) { b->front.reset(); b->back.reset(); }
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

TEST(GraphLifecycle, RechecksFreeOdomAfterEnclosingContourShrinks)
{
    struct RestoreGlobals {
        float clearance = FARUtil::kNavClearDist;
        float leaf = FARUtil::kLeafSize;
        Point3D free_odom = FARUtil::free_odom_p;
        ~RestoreGlobals() {
            FARUtil::kNavClearDist = clearance;
            FARUtil::kLeafSize = leaf;
            FARUtil::free_odom_p = free_odom;
        }
    } restore;
    FARUtil::kNavClearDist = 1.0f;
    FARUtil::kLeafSize = 0.5f;

    ContourGraph graph;
    ContourGraphParams params{};
    params.kPillarPerimeter = 1.0f;
    graph.Init(nullptr, params);
    auto odom = std::make_shared<NavNode>();
    odom->position = Point3D(0.0f, 0.0f, 0.0f);
    const PointStack enclosing{{-2.0f, -2.0f, 0.0f}, {2.0f, -2.0f, 0.0f},
                               {2.0f, 2.0f, 0.0f}, {-2.0f, 2.0f, 0.0f}};
    graph.UpdateContourGraph(odom, {enclosing});
    EXPECT_TRUE(FARUtil::PointInsideAPoly(enclosing, FARUtil::free_odom_p));

    const PointStack narrowed{{-0.25f, -0.25f, 0.0f}, {0.25f, -0.25f, 0.0f},
                              {0.25f, 0.25f, 0.0f}, {-0.25f, 0.25f, 0.0f}};
    graph.UpdateContourGraph(odom, {narrowed});
    EXPECT_FALSE(FARUtil::PointInsideAPoly(narrowed, FARUtil::free_odom_p));
    graph.ResetCurrentContour();
}

class TerrainVoteLifecycle : public ::testing::Test {
protected:
    void SetUp() override {
#ifdef FAR_PLANNER_ROS1
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "terrain_vote_lifecycle_test");
        }
#endif
        graph_ = std::make_unique<DynamicGraph>();
        graph_->ResetCurrentGraph();
    }

    void TearDown() override { graph_->ResetCurrentGraph(); }

    NavNodePtr AddNode(float x, bool is_goal = false) {
        NavNodePtr node;
        DynamicGraph::CreateNavNodeFromPoint(Point3D(x, 0.0f, 0.0f), node,
                                             false, false, is_goal);
        DynamicGraph::AddNodeToGraph(node);
        return node;
    }

    std::unique_ptr<DynamicGraph> graph_;
};

TEST_F(TerrainVoteLifecycle, DeletingMergedNodeClearsVotesWithoutVisibilityEdges) {
    const auto removed = AddNode(0.0f);
    const auto first = AddNode(1.0f);
    const auto second = AddNode(2.0f);
    DynamicGraph::RemoveInvaildTerrainConnect(removed, first);
    DynamicGraph::RemoveInvaildTerrainConnect(removed, second);
    DynamicGraph::RemoveInvaildTerrainConnect(first, second);
    ASSERT_TRUE(removed->connect_nodes.empty());
    ASSERT_TRUE(removed->potential_edges.empty());
    removed->is_merged = true;
    GraphVoteTestAccess::ClearMerged(*graph_);
    ASSERT_EQ(graph_->GetNavGraph().size(), 2U);
    EXPECT_EQ(DynamicGraph::MappedNavNodeFromId(removed->id), nullptr);
    EXPECT_TRUE(removed->terrain_votes.empty());
    ASSERT_EQ(first->terrain_votes.size(), 1U);
    ASSERT_EQ(second->terrain_votes.size(), 1U);
    EXPECT_EQ(first->terrain_votes.at(second->id), 1);
    EXPECT_EQ(second->terrain_votes.at(first->id), 1);
}

TEST_F(TerrainVoteLifecycle, GoalReplacementDoesNotAccumulateTerrainHistory) {
    const auto neighbor = AddNode(1.0f);
    for (int replacement = 0; replacement < 32; ++replacement) {
        const auto goal = AddNode(0.0f, true);
        DynamicGraph::RemoveInvaildTerrainConnect(goal, neighbor);
        DynamicGraph::ClearGoalNodeInGraph(goal);
        ASSERT_EQ(graph_->GetNavGraph().size(), 1U);
        EXPECT_TRUE(goal->terrain_votes.empty());
        EXPECT_TRUE(neighbor->terrain_votes.empty());
    }
}

class TrajectoryExtraction : public ::testing::Test {
protected:
    void SetUp() override {
#ifdef FAR_PLANNER_ROS1
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "trajectory_extraction_test");
        }
#endif
        saved_free_position_ = FARUtil::free_odom_p;
        saved_clearance_ = FARUtil::kNavClearDist;
        saved_near_distance_ = FARUtil::kNearDist;
        saved_debug_ = FARUtil::IsDebug;
        FARUtil::free_odom_p = Point3D(1.0f, 2.0f, 0.75f);
        FARUtil::kNavClearDist = 0.6f;
        FARUtil::kNearDist = 0.8f;
        FARUtil::IsDebug = false;
        graph_ = std::make_unique<DynamicGraph>();
        graph_->ResetCurrentGraph();
        DynamicGraph::CreateNavNodeFromPoint(FARUtil::free_odom_p, odom_, true);
        DynamicGraph::AddNodeToGraph(odom_);
        GraphVoteTestAccess::SetTrajectoryState(*graph_, odom_, nullptr, {},
                                               FARUtil::free_odom_p, false);
    }

    void TearDown() override {
        graph_->ResetCurrentGraph();
        FARUtil::free_odom_p = saved_free_position_;
        FARUtil::kNavClearDist = saved_clearance_;
        FARUtil::kNearDist = saved_near_distance_;
        FARUtil::IsDebug = saved_debug_;
    }

    std::unique_ptr<DynamicGraph> graph_;
    NavNodePtr odom_;
    Point3D saved_free_position_;
    float saved_clearance_, saved_near_distance_;
    bool saved_debug_;
};

TEST_F(TrajectoryExtraction, EmptyContourInputCanInitializeTrajectoryWithoutRepeatingIt) {
    ASSERT_TRUE(graph_->ExtractGraphNodes({}));
    ASSERT_EQ(graph_->GetNewNodes().size(), 1U);
    const auto trajectory = graph_->GetNewNodes().front();
    EXPECT_TRUE(trajectory->is_navpoint);
    EXPECT_FALSE(trajectory->is_odom);
    EXPECT_EQ(trajectory->position, FARUtil::free_odom_p);
    DynamicGraph::AddNodeToGraph(trajectory);
    GraphVoteTestAccess::SetTrajectoryState(*graph_, odom_, trajectory, {trajectory},
                                           trajectory->position, false);
    EXPECT_FALSE(graph_->ExtractGraphNodes({}));
    EXPECT_TRUE(graph_->GetNewNodes().empty());
}

TEST_F(TrajectoryExtraction, EmptyContourInputCanCreateARequestedBridgePoint) {
    NavNodePtr current;
    DynamicGraph::CreateNavNodeFromPoint(Point3D(0.0f, 0.0f, 0.75f), current, false, true);
    DynamicGraph::AddNodeToGraph(current);
    const Point3D last_position(3.0f, 0.0f, 0.75f);
    FARUtil::free_odom_p = Point3D(4.0f, 0.0f, 0.75f);
    GraphVoteTestAccess::SetTrajectoryState(*graph_, odom_, current, {current},
                                           last_position, true);
    ASSERT_TRUE(graph_->ExtractGraphNodes({}));
    ASSERT_EQ(graph_->GetNewNodes().size(), 1U);
    EXPECT_TRUE(graph_->GetNewNodes().front()->is_navpoint);
    EXPECT_EQ(graph_->GetNewNodes().front()->position, last_position);
}
