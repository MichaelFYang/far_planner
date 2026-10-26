/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang
 * SPDX-License-Identifier: MIT
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/graph_msger.h"
#include <visibility_graph_msg/graph_validation.h>

#include <utility>

/***************************************************************************************/

void GraphMsger::Init(const rclcpp::Node::SharedPtr nh, const GraphMsgerParams& params) {
    nh_ = nh;
    gm_params_ = params;
    graph_pub_ = nh_->create_publisher<visibility_graph_msg::msg::Graph>("/robot_vgraph", 5);
    graph_sub_ = nh_->create_subscription<visibility_graph_msg::msg::Graph>("/decoded_vgraph", 5, std::bind(&GraphMsger::GraphCallBack, this, std::placeholders::_1));

    global_graph_.clear();
    graph_node_index_.Build(global_graph_);
}

void GraphMsger::EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::msg::Graph& graphOut) {
    graphOut.nodes.clear();
    const std::string frame_id = graphOut.header.frame_id;
    for (const auto& node_ptr : graphIn) {
        if (node_ptr->connect_nodes.empty() || !IsEncodeType(node_ptr)) continue;
        visibility_graph_msg::msg::Node msg_node;
        msg_node.header.frame_id = frame_id;
        msg_node.header.stamp = graphOut.header.stamp;
        msg_node.position    = FARUtil::Point3DToGeoMsgPoint(node_ptr->position);
        msg_node.id          = node_ptr->id;
        msg_node.freetype    = static_cast<int>(node_ptr->free_direct);
        msg_node.is_covered  = node_ptr->is_covered;
        msg_node.is_frontier = node_ptr->is_frontier;
        msg_node.is_navpoint = node_ptr->is_navpoint;
        msg_node.is_boundary = node_ptr->is_boundary;
        msg_node.surface_dirs.push_back(FARUtil::Point3DToGeoMsgPoint(node_ptr->surf_dirs.first));
        msg_node.surface_dirs.push_back(FARUtil::Point3DToGeoMsgPoint(node_ptr->surf_dirs.second));
        // Encode connections
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.connect_nodes.push_back(cnode_ptr->id);
        }
        for (const auto& cnode_ptr : node_ptr->poly_connects) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.poly_connects.push_back(cnode_ptr->id);
        }
        for (const auto& cnode_ptr : node_ptr->contour_connects) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.contour_connects.push_back(cnode_ptr->id);
        }
        for (const auto& cnode_ptr : node_ptr->trajectory_connects) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.trajectory_connects.push_back(cnode_ptr->id);
        }
        graphOut.nodes.push_back(std::move(msg_node));
    }
    graphOut.size = graphOut.nodes.size();
}

void GraphMsger::UpdateGlobalGraph(const NodePtrStack& graph) {
    global_graph_ = graph;
    graph_node_index_.Build(global_graph_);
    this->PublishGlobalGraph(global_graph_);
}

void GraphMsger::PublishGlobalGraph(const NodePtrStack& graphIn) {
    visibility_graph_msg::msg::Graph graph_msg;
    graph_msg.header.frame_id = gm_params_.frame_id;
    graph_msg.header.stamp = nh_->now();
    graph_msg.robot_id = gm_params_.robot_id;
    this->EncodeGraph(graphIn, graph_msg);
    graph_pub_->publish(graph_msg);
}

bool GraphMsger::ValidateGraphMessage(const visibility_graph_msg::msg::Graph& graph,
                                       const std::string& expected_frame, std::string& error) {
    return graph_message_validation::ValidateGraph(
        graph, expected_frame, [](const auto& node) { return node.freetype; }, error);
}

void GraphMsger::GraphCallBack(const visibility_graph_msg::msg::Graph::SharedPtr msg) {
    if (msg->nodes.empty()) return;
    std::string error;
    if (!ValidateGraphMessage(*msg, gm_params_.frame_id, error)) {
        RCLCPP_WARN(nh_->get_logger(), "Ignoring decoded graph: %s", error.c_str());
        return;
    }
    NodePtrStack decoded_nodes;
    IdxMap nodeIdx_idx_map;
    decoded_nodes.reserve(msg->nodes.size());
    nodeIdx_idx_map.reserve(msg->nodes.size());
    for (std::size_t i=0; i<msg->nodes.size(); i++) {
        const auto& node = msg->nodes[i];
        const Point3D node_p = Point3D(node.position.x, node.position.y, node.position.z);
        NavNodePtr nearest_node_ptr = NearestNodePtrOnGraph(node_p, gm_params_.dist_margin);
        if (nearest_node_ptr == NULL || IsMismatchFreeNode(nearest_node_ptr, node)) {
            CreateDecodedNavNode(node, nearest_node_ptr);
            DynamicGraph::AddNodeToGraph(nearest_node_ptr);
        }
        decoded_nodes.push_back(nearest_node_ptr);
        nodeIdx_idx_map.insert({node.id, i});
    }
    // Assign connections with fully connection votes
    for (std::size_t i=0; i<msg->nodes.size(); i++) {
        const auto& node = msg->nodes[i];
        const NavNodePtr node_ptr = decoded_nodes[i];
        // graph connections
        NavNodePtr cnode_ptr = NULL;
        for (const auto& cid : node.connect_nodes) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::AddEdge(node_ptr, cnode_ptr);
            }
        }
        // poly connections
        for (const auto& cid : node.poly_connects) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::FillPolygonEdgeConnect(node_ptr, cnode_ptr, gm_params_.votes_size);
            }
        }
        // contour connections
        for (const auto& cid : node.contour_connects) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::FillContourConnect(node_ptr, cnode_ptr, gm_params_.votes_size);
            }
        }
        // trajectory connection
        for (const auto& cid : node.trajectory_connects) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::FillTrajConnect(node_ptr, cnode_ptr);
            }
        }
    }
}

NavNodePtr GraphMsger::NearestNodePtrOnGraph(const Point3D p, const float radius) {
    const auto node = graph_node_index_.Nearest(p, radius);
    return node && DynamicGraph::MappedNavNodeFromId(node->id) == node ? node : nullptr;
}

void GraphMsger::CreateDecodedNavNode(const visibility_graph_msg::msg::Node& vnode, NavNodePtr& node_ptr) {
    const Point3D p = Point3D(vnode.position.x, vnode.position.y, vnode.position.z);
    const bool is_covered  = vnode.is_covered  == 0 ? false : true;
    const bool is_frontier = vnode.is_frontier == 0 ? false : true;
    const bool is_navpoint = vnode.is_navpoint == 0 ? false : true;
    const bool is_boundary = vnode.is_boundary == 0 ? false : true;
    DynamicGraph::CreateNavNodeFromPoint(p, node_ptr, false, is_navpoint, false, is_boundary);
    node_ptr->is_active = false;
    /* Assign relative values */
    node_ptr->is_covered  = is_covered;
    node_ptr->is_frontier = is_frontier;
    DynamicGraph::FillFrontierVotes(node_ptr, is_frontier);
    // positions
    node_ptr->is_finalized = true;
    const std::deque<Point3D> pos_queue(gm_params_.pool_size, p);
    node_ptr->pos_filter_vec = pos_queue;
    const PointPair surf_pair = {Point3D(vnode.surface_dirs[0].x, vnode.surface_dirs[0].y, vnode.surface_dirs[0].z),
                                 Point3D(vnode.surface_dirs[1].x, vnode.surface_dirs[1].y, vnode.surface_dirs[1].z)};
    // surf directions
    node_ptr->free_direct = static_cast<NodeFreeDirect>(vnode.freetype);
    node_ptr->surf_dirs = surf_pair;
    const std::deque<PointPair> surf_queue(gm_params_.pool_size, surf_pair);
    node_ptr->surf_dirs_vec = surf_queue;
}

